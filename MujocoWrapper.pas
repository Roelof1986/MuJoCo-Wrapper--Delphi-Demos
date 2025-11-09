unit MujocoWrapper;

interface

type
  TMJWHandle = Pointer;

{ basis / lifecycle }
function  MJW_GetLastError: PAnsiChar; cdecl; external 'mujoco_delphi_wrapper.dll';
function  MJW_Create(modelXML: PAnsiChar; width, height: LongInt): TMJWHandle; cdecl; external 'mujoco_delphi_wrapper.dll';
procedure MJW_Destroy(h: TMJWHandle); cdecl; external 'mujoco_delphi_wrapper.dll';

{ main loop }
procedure MJW_Step(h: TMJWHandle); cdecl; external 'mujoco_delphi_wrapper.dll';
procedure MJW_Render(h: TMJWHandle); cdecl; external 'mujoco_delphi_wrapper.dll';
procedure MJW_PollEvents(h: TMJWHandle); cdecl; external 'mujoco_delphi_wrapper.dll';
function  MJW_ShouldClose(h: TMJWHandle): LongInt; cdecl; external 'mujoco_delphi_wrapper.dll';

{ sim params }
procedure MJW_SetTimestep(h: TMJWHandle; dt: Double); cdecl; external 'mujoco_delphi_wrapper.dll';
procedure MJW_Reset(h: TMJWHandle); cdecl; external 'mujoco_delphi_wrapper.dll';
//function  MJW_GetSimTime(h: TMJWHandle): Double; cdecl; external 'mujoco_delphi_wrapper.dll';

{ io / sizes }
//procedure MJW_GetSizes(h: TMJWHandle; var nq, nv, nu: LongInt); cdecl; external 'mujoco_delphi_wrapper.dll';
function  MJW_GetNQ(h: TMJWHandle): LongInt; cdecl; external 'mujoco_delphi_wrapper.dll';
function  MJW_GetNV(h: TMJWHandle): LongInt; cdecl; external 'mujoco_delphi_wrapper.dll';
function  MJW_GetNU(h: TMJWHandle): LongInt; cdecl; external 'mujoco_delphi_wrapper.dll';

procedure MJW_SetCtrl(h: TMJWHandle; const ctrl: PDouble; len: LongInt); cdecl; external 'mujoco_delphi_wrapper.dll';
function  MJW_GetCtrl(h: TMJWHandle; outBuf: PDouble; len: LongInt): LongInt; cdecl; external 'mujoco_delphi_wrapper.dll';
function  MJW_GetQpos(h: TMJWHandle; outBuf: PDouble; len: LongInt): LongInt; cdecl; external 'mujoco_delphi_wrapper.dll';
function  MJW_GetQvel(h: TMJWHandle; outBuf: PDouble; len: LongInt): LongInt; cdecl; external 'mujoco_delphi_wrapper.dll';

{ camera / window }
procedure MJW_SetCameraFree(h: TMJWHandle; dist, azim, elev, lookX, lookY, lookZ: Double); cdecl; external 'mujoco_delphi_wrapper.dll';
procedure MJW_ShowAndPos(h: TMJWHandle; x, y: LongInt); cdecl; external 'mujoco_delphi_wrapper.dll';

{ forces / torques }
function  MJW_GetActuatorForce(h: TMJWHandle; outBuf: PDouble; len: LongInt): LongInt; cdecl; external 'mujoco_delphi_wrapper.dll';
function  MJW_GetQfrcActuator(h: TMJWHandle; outBuf: PDouble; len: LongInt): LongInt; cdecl; external 'mujoco_delphi_wrapper.dll';

procedure MJW_ApplyDof(h: TMJWHandle; dofIndex: LongInt; value: Double); cdecl; external 'mujoco_delphi_wrapper.dll';
procedure MJW_ClearApplied(h: TMJWHandle); cdecl; external 'mujoco_delphi_wrapper.dll';

function  MJW_JointNameToDof(h: TMJWHandle; jointName: PAnsiChar; var firstDof, ndof: LongInt): LongInt; cdecl; external 'mujoco_delphi_wrapper.dll';

// proprioception

function MJW_Joint1D_Angle(h: TMJWHandle; jointName: PAnsiChar): Double; cdecl; external 'mujoco_delphi_wrapper.dll';
function MJW_Joint1D_Vel  (h: TMJWHandle; jointName: PAnsiChar): Double; cdecl; external 'mujoco_delphi_wrapper.dll';

function MJW_GetJoint1D_State(h: TMJWHandle; jointName: PAnsiChar;
                              var angle, velocity: Double): LongInt; cdecl; external 'mujoco_delphi_wrapper.dll';

function MJW_GetJointState(h: TMJWHandle; jointName: PAnsiChar;
                           angleBuf: PDouble; velBuf: PDouble; maxLen: LongInt): LongInt; cdecl; external 'mujoco_delphi_wrapper.dll';

function MJW_DebugJoint1D(h: TMJWHandle; jointName: PAnsiChar;
                          var jtype, qposAdr, dofAdr: LongInt): LongInt; cdecl; external 'mujoco_delphi_wrapper.dll';

//function MJW_Version: PAnsiChar; cdecl; external 'mujoco_delphi_wrapper.dll';
//function MJW_Ping(x: LongInt): LongInt; cdecl; external 'mujoco_delphi_wrapper.dll';

procedure MJW_DumpAllJoints(h: TMJWHandle); cdecl; external 'mujoco_delphi_wrapper.dll';

//begin
//  MJW_DumpAllJoints(H);
//end;

// ===== Joint Introspection & Live Data =====

function MJW_GetJointCount(h: TMJWHandle): LongInt;
  cdecl; external 'mujoco_delphi_wrapper.dll';

function MJW_GetJointName(h: TMJWHandle; jindex: LongInt): PAnsiChar;
  cdecl; external 'mujoco_delphi_wrapper.dll';

function MJW_GetJointInfo(h: TMJWHandle; jindex: LongInt;
                          var jtype, qposAdr, dofAdr, ndof: LongInt): LongInt;
  cdecl; external 'mujoco_delphi_wrapper.dll';

function MJW_GetJointValuesByIndex(h: TMJWHandle; jindex: LongInt;
                                   angleBuf: PDouble; velBuf: PDouble;
                                   maxLen: LongInt): LongInt;
  cdecl; external 'mujoco_delphi_wrapper.dll';

// ===== Extra hulpmiddelen (optioneel) =====

// huidige simulatie-tijd
function MJW_GetSimTime(h: TMJWHandle): Double;
  cdecl; external 'mujoco_delphi_wrapper.dll';

// voor debug (de versie-string)
function MJW_Version: PAnsiChar;
  cdecl; external 'mujoco_delphi_wrapper.dll';

// eenvoudige ping om te checken of de DLL geladen is
function MJW_Ping(x: LongInt): LongInt;
  cdecl; external 'mujoco_delphi_wrapper.dll';

function MJW_ReadByAdr(h: TMJWHandle; qposAdr, dofAdr: LongInt;
                       var angleOut, velOut: Double): LongInt; cdecl; external 'mujoco_delphi_wrapper.dll';

procedure MJW_GetSizes(h: TMJWHandle; var nq, nv: LongInt); cdecl; external 'mujoco_delphi_wrapper.dll';
function  MJW_ReadQpos(h: TMJWHandle; idx: LongInt): Double; cdecl; external 'mujoco_delphi_wrapper.dll';
function  MJW_ReadQvel(h: TMJWHandle; idx: LongInt): Double; cdecl; external 'mujoco_delphi_wrapper.dll';

//procedure MJW_CamOrbit(h: TMJWHandle; dx, dy, scale: Double); cdecl; external 'mujoco_delphi_wrapper.dll';
//procedure MJW_CamPan  (h: TMJWHandle; dx, dy, scale: Double); cdecl; external 'mujoco_delphi_wrapper.dll';
//procedure MJW_CamZoom (h: TMJWHandle; wheelDelta, scale: Double); cdecl; external 'mujoco_delphi_wrapper.dll';

procedure MJW_EnableMouseCam(h: Pointer; enable: LongInt); cdecl; external 'mujoco_delphi_wrapper.dll';
procedure MJW_SetMouseCamParams(h: Pointer; rotScale, panScale, zoomScale: Double); cdecl; external 'mujoco_delphi_wrapper.dll';

implementation
end.
