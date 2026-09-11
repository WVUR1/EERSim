"""Download only the matching public PhysX header closure; no build/install/GPU use."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import sys
import urllib.request


TAG = "ovphysx-0.5.11"
PREFIX = "physx/include/"
BASE = f"https://raw.githubusercontent.com/NVIDIA-Omniverse/PhysX/{TAG}/"


def fetch(path):
    with urllib.request.urlopen(BASE + PREFIX + path, timeout=30) as response:
        return path, response.read()


def main():
    destination = Path(sys.argv[1]).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(
        f"https://api.github.com/repos/NVIDIA-Omniverse/PhysX/git/trees/{TAG}?recursive=1",
        timeout=30,
    ) as response:
        tree = json.load(response)
    if tree.get("truncated"):
        raise RuntimeError("GitHub returned an incomplete source tree")
    available = {
        entry["path"][len(PREFIX):] for entry in tree["tree"]
        if entry["type"] == "blob" and entry["path"].startswith(PREFIX)
    }
    pending = {
        "PxScene.h", "PxRigidDynamic.h", "PxRigidStatic.h", "PxArticulationReducedCoordinate.h",
        "PxArticulationLink.h", "PxShape.h", "PxSimulationStatistics.h",
        "geometry/PxConvexMeshGeometry.h", "geometry/PxConvexMesh.h",
        "geometry/PxTriangleMeshGeometry.h", "geometry/PxTriangleMesh.h",
        "cudamanager/PxCudaContextManager.h", "task/PxCpuDispatcher.h",
    }
    downloaded = {}
    unresolved = set()
    with ThreadPoolExecutor(max_workers=8) as pool:
        while pending:
            batch = sorted(pending - downloaded.keys())
            pending.clear()
            if not batch:
                break
            if len(downloaded) + len(batch) > 500:
                raise RuntimeError("Unexpectedly large public-header closure")
            for path, data in pool.map(fetch, batch):
                target = destination.joinpath(*PurePosixPath(path).parts)
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(data)
                downloaded[path] = hashlib.sha256(data).hexdigest()
                for reference in re.findall(r'^\s*#\s*include\s*"([^"]+)"', data.decode(), re.M):
                    relative = str(PurePosixPath(path).parent / reference)
                    candidate = relative if relative in available else reference
                    if candidate in available:
                        if candidate not in downloaded:
                            pending.add(candidate)
                    else:
                        unresolved.add(reference)
    manifest = {"tag": TAG, "source": BASE + PREFIX, "sha256": downloaded,
                "non_public_includes": sorted(unresolved)}
    (destination / "headers-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Downloaded {len(downloaded)} public headers from {TAG} to {destination}")
    if unresolved:
        print("System/generated includes:", ", ".join(sorted(unresolved)))


if __name__ == "__main__":
    main()
