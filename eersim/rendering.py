"""GPU OpenGL visualization of an existing Bullet world, without another GUI.

The simulation thread owns this renderer and its OpenGL context. Geometry is
uploaded once; subsequent frames update link transforms and the camera. Physics
and collision geometry remain entirely in Bullet. The caller handles fallback
when VTK, an OpenGL driver, or an imported visual format is unavailable.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
import threading
from typing import Iterable

import numpy as np
import pybullet as p


def _matrix(position, quaternion) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = np.asarray(p.getMatrixFromQuaternion(quaternion)).reshape(3, 3)
    result[:3, 3] = position
    return result


def _inverse_rigid(matrix: np.ndarray) -> np.ndarray:
    """Rigid transforms need only a transpose, not a LAPACK matrix solve."""
    result = np.eye(4)
    result[:3, :3] = matrix[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ matrix[:3, 3]
    return result


def _material_palette(path: Path, libraries: list[str]) -> dict[str, tuple]:
    palette = {}
    for library in libraries:
        material_path = path.parent / library
        if not material_path.is_file():
            continue
        current = None
        for line in material_path.read_text(encoding="utf-8-sig").splitlines():
            tokens = line.split()
            if not tokens or tokens[0].startswith("#"):
                continue
            if tokens[0] == "newmtl":
                current = " ".join(tokens[1:])
                palette[current] = [1., 1., 1., 1.]
            elif current and tokens[0] == "Kd":
                palette[current][:3] = [float(x) for x in tokens[1:4]]
            elif current and tokens[0] in ("d", "Tr"):
                opacity = float(tokens[1])
                palette[current][3] = 1 - opacity if tokens[0] == "Tr" else opacity
            elif tokens[0].lower().startswith("map_"):
                raise ValueError(f"GPU renderer does not yet support OBJ texture maps: {material_path}")
    return palette


@dataclass
class _Visual:
    actor: object
    body: int
    link: int
    local: np.ndarray
    matrix: object


class HardwareRenderer:
    """Offscreen VTK/OpenGL renderer. Construct, render and close on one thread.

    ``base_inertial_frames`` maps body IDs to local ``(position, quaternion)``
    frames captured when each body was loaded. It is required for Bullet rigid
    bodies, whose getDynamicsInfo inertial-frame fields are not initialized in
    some supported engine versions. ``physics_api`` supplies authoritative live
    poses. ``visual_shapes`` can supply immutable Bullet-format descriptors
    directly, with captured inertial frames for every body, without any Bullet
    scene. Otherwise visual geometry is read from the non-stepping Bullet scene.
    """

    def __init__(self, client: int, body_ids: Iterable[int],
                 base_inertial_frames: dict[int, tuple] | None = None, physics_api=None,
                 visual_shapes=None):
        import vtk
        from vtk.util.numpy_support import vtk_to_numpy

        self.vtk = vtk
        self._vtk_to_numpy = vtk_to_numpy
        self.client = client
        self._physics = p if physics_api is None else physics_api
        self._owner = threading.get_ident()
        self._closed = False
        self._geometry = {}
        self._mappers = {}
        self._visuals: list[_Visual] = []
        self._base_inertials = {}
        self.renderer_name = "OpenGL initialization pending"
        self.vendor = ""
        self.capabilities = ""
        self.hardware = False
        self.status = {"backend": "vtk-opengl", "renderer": self.renderer_name,
                       "vendor": "", "version": "", "hardware": False,
                       "hardware_accelerated": False, "device": self.renderer_name}
        body_ids = tuple(body_ids)
        descriptors = None
        if visual_shapes is not None:
            descriptors = {body: [] for body in body_ids}
            if base_inertial_frames is None or any(body not in base_inertial_frames for body in body_ids):
                raise ValueError("Descriptor rendering needs an inertial frame for every body")
            for shape in visual_shapes:
                if shape[0] not in descriptors:
                    raise ValueError("Visual descriptor refers to an unknown body")
                descriptors[shape[0]].append(shape)
        self._renderer = vtk.vtkRenderer()
        self._renderer.SetBackground(9 / 255, 20 / 255, 32 / 255)
        self._renderer.SetBackgroundAlpha(1.)
        self._renderer.SetUseFXAA(True)
        self._window = vtk.vtkRenderWindow()
        self._window.SetOffScreenRendering(1)
        self._window.SetShowWindow(False)
        self._window.SetMultiSamples(0)
        self._window.SetAlphaBitPlanes(1)
        self._window.SetSize(64, 64)
        self._window.AddRenderer(self._renderer)
        self._capture = vtk.vtkWindowToImageFilter()
        self._capture.SetInput(self._window)
        self._capture.SetInputBufferTypeToRGBA()
        self._capture.ReadFrontBufferOff()
        self._capture.ShouldRerenderOff()
        self._camera = self._renderer.GetActiveCamera()
        self._camera.SetViewAngle(52)
        self._camera.SetClippingRange(.02, 100)
        # Camera-relative fill makes imported machinery legible from any orbit.
        key = vtk.vtkLight()
        key.SetLightTypeToSceneLight()
        key.SetPosition(-3, -4, 7)
        key.SetFocalPoint(0, 0, 0)
        key.SetIntensity(.85)
        self._renderer.AddLight(key)
        fill = vtk.vtkLight()
        fill.SetLightTypeToHeadlight()
        fill.SetIntensity(.50)
        self._renderer.AddLight(fill)
        try:
            for body in body_ids:
                if base_inertial_frames is not None and body in base_inertial_frames:
                    inertial_position, inertial_rotation = base_inertial_frames[body]
                else:
                    inertia = p.getDynamicsInfo(body, -1, physicsClientId=client)
                    if len(inertia) > 10 and inertia[10] == 1:
                        # Bullet 3.2.5 can leave this frame uninitialized for
                        # maximal-coordinate bodies. Use the captured frame.
                        raise ValueError(f"GPU rendering needs the captured base inertial frame for maximal-coordinate body {body}")
                    inertial_position, inertial_rotation = inertia[3], inertia[4]
                if (len(inertial_position) != 3 or len(inertial_rotation) != 4 or
                        not np.isfinite([*inertial_position, *inertial_rotation]).all()):
                    raise ValueError(f"Body {body} has an invalid captured base inertial frame")
                self._base_inertials[body] = _inverse_rigid(_matrix(inertial_position, inertial_rotation))
                shapes = descriptors[body] if descriptors is not None else p.getVisualShapeData(
                    body, flags=p.VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS, physicsClientId=client) or ()
                for shape in shapes:
                    self._add_shape(shape)
        except Exception:
            self.close()
            raise

    def _check_thread(self):
        if threading.get_ident() != self._owner:
            raise RuntimeError("The OpenGL renderer must be used and closed by its owning simulation thread")
        if self._closed:
            raise RuntimeError("The OpenGL renderer is closed")

    def _read_mesh(self, path: Path):
        vtk = self.vtk
        readers = {".obj": vtk.vtkOBJReader, ".stl": vtk.vtkSTLReader, ".ply": vtk.vtkPLYReader}
        reader_type = readers.get(path.suffix.lower())
        if reader_type is None:
            raise ValueError(f"GPU renderer supports OBJ, STL and PLY meshes; unsupported visual: {path}")
        if not path.is_file():
            raise ValueError(f"GPU visual mesh does not exist: {path}")
        reader = reader_type()
        reader.SetFileName(str(path))
        reader.Update()
        poly = vtk.vtkPolyData()
        poly.ShallowCopy(reader.GetOutput())
        if not poly.GetNumberOfPoints() or not poly.GetNumberOfCells():
            raise ValueError(f"GPU visual mesh has no renderable geometry: {path}")
        libraries = poly.GetFieldData().GetAbstractArray("MaterialLibraries")
        _material_palette(path, [libraries.GetValue(i) for i in range(libraries.GetNumberOfValues())]
                          if libraries else [path.with_suffix(".mtl").name])
        # Bullet resolves each visual's final color when it loads the URDF.
        # Reinterpreting OBJ material IDs here would override explicit URDF
        # colors and disagree with the CPU camera, even for a white tint.
        poly.GetCellData().SetScalars(None)
        # VTK's reader retains CAD normals. Generate them only for meshes that
        # omit normals, avoiding repeated work for many instances of one part.
        if poly.GetPointData().GetNormals() is None:
            normals = vtk.vtkPolyDataNormals()
            normals.SetInputData(poly)
            normals.SetFeatureAngle(45)
            normals.ConsistencyOn()
            normals.Update()
            out = vtk.vtkPolyData()
            out.ShallowCopy(normals.GetOutput())
            poly = out
        return poly

    def _primitive(self, kind, dimensions):
        vtk = self.vtk
        if kind == p.GEOM_BOX:
            source = vtk.vtkCubeSource()
            source.SetXLength(dimensions[0])
            source.SetYLength(dimensions[1])
            source.SetZLength(dimensions[2])
        elif kind == p.GEOM_SPHERE:
            source = vtk.vtkSphereSource()
            source.SetRadius(dimensions[0])
            source.SetThetaResolution(32)
            source.SetPhiResolution(24)
        elif kind in (p.GEOM_CYLINDER, p.GEOM_CAPSULE):
            source = vtk.vtkCylinderSource()
            source.SetHeight(dimensions[0])
            source.SetRadius(dimensions[1])
            source.SetResolution(40)
            source.SetCapping(kind != p.GEOM_CAPSULE)
            rotate = vtk.vtkTransform()
            rotate.RotateX(90)
            transform = vtk.vtkTransformPolyDataFilter()
            transform.SetInputConnection(source.GetOutputPort())
            transform.SetTransform(rotate)
            transform.Update()
            if kind == p.GEOM_CAPSULE:
                append = vtk.vtkAppendPolyData()
                append.AddInputData(transform.GetOutput())
                for sign in (-1, 1):
                    cap = vtk.vtkSphereSource()
                    cap.SetRadius(dimensions[1])
                    cap.SetCenter(0, 0, sign * dimensions[0] / 2)
                    cap.SetThetaResolution(32)
                    cap.SetPhiResolution(16)
                    cap.SetStartPhi(0 if sign > 0 else 90)
                    cap.SetEndPhi(90 if sign > 0 else 180)
                    cap.Update()
                    append.AddInputData(cap.GetOutput())
                append.Update()
                return append.GetOutput()
            return transform.GetOutput()
        elif kind == p.GEOM_PLANE:
            source = vtk.vtkPlaneSource()
            source.SetOrigin(-100, -100, 0)
            source.SetPoint1(100, -100, 0)
            source.SetPoint2(-100, 100, 0)
        else:
            raise ValueError(f"GPU renderer does not support Bullet visual geometry type {kind}")
        source.Update()
        return source.GetOutput()

    def _add_shape(self, shape):
        vtk = self.vtk
        body, link, kind, dims, filename, pos, quat, rgba = shape[:8]
        if rgba[3] <= 0:
            return
        if len(shape) > 8 and shape[8] >= 0:
            raise ValueError("GPU rendering of Bullet runtime textures is not supported yet")
        if kind == p.GEOM_MESH:
            path = Path(filename.decode() if isinstance(filename, bytes) else filename).resolve()
            key = (kind, str(path))
            if key not in self._geometry:
                self._geometry[key] = self._read_mesh(path)
        else:
            key = (kind, tuple(dims))
            if key not in self._geometry:
                self._geometry[key] = self._primitive(kind, dims)
        if key not in self._mappers:
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(self._geometry[key])
            if self._geometry[key].GetCellData().GetScalars() is not None:
                mapper.SetScalarModeToUseCellData()
                mapper.SetColorModeToDirectScalars()
                mapper.ScalarVisibilityOn()
            else:
                mapper.ScalarVisibilityOff()
            self._mappers[key] = mapper
        actor = vtk.vtkActor()
        actor.SetMapper(self._mappers[key])
        prop = actor.GetProperty()
        prop.SetColor(*rgba[:3])
        prop.SetOpacity(rgba[3])
        prop.SetAmbient(.40)
        prop.SetDiffuse(.60)
        prop.SetSpecular(.12)
        prop.SetSpecularPower(28)
        prop.SetInterpolationToPhong()
        local = _matrix(pos, quat)
        if kind == p.GEOM_MESH:
            local[:3, :3] = local[:3, :3] @ np.diag(dims)
        matrix = vtk.vtkMatrix4x4()
        actor.SetUserMatrix(matrix)
        self._visuals.append(_Visual(actor, body, link, local, matrix))
        self._renderer.AddActor(actor)

    def _update_poses(self):
        transforms = {}
        for visual in self._visuals:
            key = (visual.body, visual.link)
            if key not in transforms:
                if visual.link < 0:
                    position, rotation = self._physics.getBasePositionAndOrientation(visual.body, physicsClientId=self.client)
                    transforms[key] = _matrix(position, rotation) @ self._base_inertials[visual.body]
                else:
                    state = self._physics.getLinkState(visual.body, visual.link, computeForwardKinematics=1,
                                           physicsClientId=self.client)
                    transforms[key] = _matrix(state[4], state[5])
            matrix = transforms[key] @ visual.local
            visual.matrix.DeepCopy(matrix.ravel())

    def _read_capabilities(self):
        self.capabilities = self._window.ReportCapabilities() or ""
        def field(name):
            match = re.search(rf"^{re.escape(name)}\s*:\s*(.+)$", self.capabilities, re.MULTILINE | re.IGNORECASE)
            return match.group(1).strip() if match else "Unknown"
        self.vendor = field("OpenGL vendor string")
        self.renderer_name = field("OpenGL renderer string")
        version = field("OpenGL version string")
        device = (self.vendor + " " + self.renderer_name).lower()
        software = ("llvmpipe", "softpipe", "software rasterizer", "gdi generic",
                    "microsoft basic render", "swiftshader", "swrast", "osmesa")
        self.hardware = self.renderer_name != "Unknown" and not any(name in device for name in software)
        self.status = {"backend": "vtk-opengl", "renderer": self.renderer_name,
                       "vendor": self.vendor, "version": version, "hardware": self.hardware,
                       "hardware_accelerated": self.hardware, "device": self.renderer_name,
                       "visual_shapes": len(self._visuals), "cached_geometries": len(self._geometry)}

    def render(self, width: int, height: int, yaw: float, pitch: float,
               distance: float, target=(0., 0., 0.)) -> np.ndarray:
        self._check_thread()
        width, height = int(width), int(height)
        if width < 1 or height < 1:
            raise ValueError("Camera width and height must be positive")
        self._update_poses()
        view = np.asarray(p.computeViewMatrixFromYawPitchRoll(target, max(.1, distance), yaw, pitch, 0, 2)).reshape(4, 4, order="F")
        camera_world = _inverse_rigid(view)
        self._camera.SetPosition(*camera_world[:3, 3])
        self._camera.SetFocalPoint(*target)
        self._camera.SetViewUp(*camera_world[:3, 1])
        if self._window.GetSize() != (width, height):
            self._window.SetSize(width, height)
        self._window.Render()
        if not self.capabilities:
            self._read_capabilities()
        self._capture.Modified()
        self._capture.Update()
        scalars = self._capture.GetOutput().GetPointData().GetScalars()
        if scalars is None:
            raise RuntimeError("OpenGL camera produced no pixel buffer")
        rgba = self._vtk_to_numpy(scalars).reshape(height, width, 4)[::-1].copy()
        # The GUI consumes a fully composed frame; render-window alpha should
        # never turn the navy background transparent when Qt paints this image.
        rgba[:, :, 3] = 255
        return rgba

    def close(self):
        if self._closed:
            return
        self._check_thread()
        self._capture.SetInput(None)
        self._renderer.RemoveAllViewProps()
        self._window.Finalize()
        self._visuals.clear()
        self._mappers.clear()
        self._geometry.clear()
        self._closed = True
