; Built by build-installer.ps1 using Inno Setup 7.1 or newer.
#ifndef BundleRoot
  #error BundleRoot must identify a complete EERSim directory bundle.
#endif
#define AppVersion "0.1.6"

[Setup]
AppId={{95A78754-70D4-4D44-8E83-9241A32DDA25}
AppName=EERSim
AppVersion={#AppVersion}
AppPublisher=WVU Robotics Club
LicenseFile={#BundleRoot}\_internal\LICENSE
DefaultDirName={localappdata}\Programs\EERSim
DefaultGroupName=EERSim
PrivilegesRequired=lowest
SetupArchitecture=x64
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
MinVersion=10.0
UsePreviousAppDir=yes
DisableWelcomePage=no
DisableDirPage=no
DisableProgramGroupPage=no
WizardStyle=modern
Compression=lzma2/normal
SolidCompression=yes
UninstallDisplayIcon={app}\EERSim.exe
CloseApplications=yes
RestartApplications=no
OutputBaseFilename=EERSim-Setup-x64
#ifdef SignInstaller
SignTool=eersimrelease
SignedUninstaller=yes
#else
SignedUninstaller=no
#endif

[Tasks]
Name: "desktopicon"; Description: "Create a &desktop shortcut"; GroupDescription: "Additional shortcuts:"; Flags: unchecked

[Files]
Source: "{#BundleRoot}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{group}\EERSim"; Filename: "{app}\EERSim.exe"; WorkingDir: "{app}"
Name: "{group}\Uninstall EERSim"; Filename: "{uninstallexe}"
Name: "{autodesktop}\EERSim"; Filename: "{app}\EERSim.exe"; WorkingDir: "{app}"; Tasks: desktopicon

[Run]
Filename: "{app}\EERSim.exe"; Description: "Launch EERSim"; WorkingDir: "{app}"; Flags: nowait postinstall skipifsilent

; Standard uninstall removes installed files only. User data and external
; projects/imports are deliberately absent from the uninstall file list.
