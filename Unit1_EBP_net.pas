unit Unit1_EBP_net;

interface

uses
  System.SysUtils, System.Types, System.Classes, System.IOUtils, System.Math, System.StrUtils,
  FMX.Types, FMX.Controls, FMX.Forms, FMX.Dialogs,
  Winapi.Windows,
  MujocoWrapper;

type
  TForm1 = class(TForm)
  private
    // ---- MuJoCo handle + UI ----
    H           : TMJWHandle;
    Timer       : TTimer;
    FrameCount  : Integer;

    // ================== HIPS (glute kicks) ==================
    HipL_FirstDof : Integer;  HipL_NDof : Integer;
    HipR_FirstDof : Integer;  HipR_NDof : Integer;

    KickActive     : Boolean;
    KickStartMS    : UInt64;
    KickDurationMS : Integer;    // ms torque aan
    KickIntervalMS : Integer;    // ms tussen kicks
    KickTauL       : Double;     // Nm
    KickTauR       : Double;     // Nm

    // ================== ARMS (sinus shoulders + elbows) ==================
    ShoulderL_Dof : Integer;  ShoulderL_NDof : Integer;
    ShoulderR_Dof : Integer;  ShoulderR_NDof : Integer;
    ElbowL_Dof    : Integer;  ElbowL_NDof    : Integer;
    ElbowR_Dof    : Integer;  ElbowR_NDof    : Integer;

    ArmAmpNm   : Double;   // amplitude (Nm)
    ArmFreqHz  : Double;   // frequentie (Hz)
    ElbowGain  : Double;   // factor tov schouder
    PhaseR     : Double;   // tegenfase (rad)

    // ================== EXTRA LEDENMATEN DOF-indexen ==================
    AbdZ_Dv, AbdY_Dv, AbdX_Dv : Integer;
    KneeL_Dv, KneeR_Dv        : Integer;
    AnkleL_Dv, AnkleR_Dv      : Integer;
    WristL_Dv, WristR_Dv      : Integer;

    // ------- Teach/Predict mode -------
    type TMode = (moTeach, moPredict);

const
    InputDelayLen = {30}{24}{6}{4}{5}{17}{27}{37}{7}{9}{29}22;

    MaxForce = {300}{660}{520}500{700};

    ConsoleVizOn = False{True};

    TraceSpd = {0.011}{0.005}{0.09}0.1;

    LearningRate = {0.09}{0.02} {0.01}0.009;

    AbdMul = {3.5}{7}9;

    HipMul = {2.5}3.5;

    LegMul = 3.0;

var
    FMode: TMode;
    FTrainOnlyInTeach: Boolean;
    FSpaceWasDown: Boolean; // debouncer voor spatiebalk

    // ------- Aangestuurde DOFs (vaste volgorde) -------
    FDofs: TArray<Integer>;

    // ------- NN-config -------
    FInSize, FHidden, FOutSize : Integer;
    FLR       : Double;    // learning rate
    FOutScale : Double;    // torque scale (Nm)
    // Normalisatie van sensen
    FVelV50   : Double;    // rad/s waar vel_unit = 0.5
    FVelK     : Double;    // = ln(2)/FVelV50

    // Gewichten/buffers
    W1, B1 : TArray<Double>; // [Hidden x InSize], [Hidden]
    W2, B2 : TArray<Double>; // [Out x Hidden], [Out]
    X, Hidden, Y: TArray<Double>; // input, hidden (hernoemd), output (tanh)

    RanTrace, TauTrace : TArray<Double>;

    OutW : TArray<Double>;

    ObsOut, PreObsOut : TArray<Double>;

    PrevObsOut : array[0..InputDelayLen] of TArray<Double>;

    SinCycle, Cyc : Longint;

    // ---------------- helpers ----------------
    function  RandSym(const Amp: Double): Double;
    function  Find1Dof(const nameL, nameR: string;
                       out dofL, ndofL, dofR, ndofR: Integer): Boolean;
    function  FindHipsFallback: Boolean;
    function  FindShouldersAndElbows: Boolean;

    function  MapDof(const nm: AnsiString): Integer;
    procedure MapExtraDOFs;  // abdomen/knees/ankles/wrists
    procedure BuildControlledDofs; // vult FDofs in vaste volgorde

    // sim
    procedure StartSim(const ModelPath: string);
    procedure OnTick(Sender: TObject);

    // --- Input/Teacher/Apply ---
    procedure ReadObs(out Obs: TArray<Double>); // per DOF: angle_unit, vel_unit, vel_sign + tijd
    procedure ComputeTeacher(const t: Double; out TauTeacher: TArray<Double>);
    procedure ApplyForcesVector(const Tau: TArray<Double>);

    // --- Normalisatie helpers ---
    function WrapPi(a: Double): Double; {inline}
    function AngleRadToUnit(a: Double): Double; {inline}      // 0..1
    function VelToUnitAbs(v, k: Double): Double; {inline}     // 0..1
    function DvToQp(const dv: Integer): Integer; {inline}     // qvel->qpos index

    // --- NN ---
    function  Idx(OutIdx, InIdx, InSize: Integer): Integer; {inline}
    class function Tanh_(v: Double): Double; static; {inline}
    class function DTanh_(post: Double): Double; static; {inline}

    procedure InitNetwork(InSize, Hidden, OutSize: Integer; Seed: UInt32 = 1337);
    procedure PropagateForward(const Input: TArray<Double>; out Output: TArray<Double>);
    function  PropagateBack(const Input, TargetNm: TArray<Double>; LR: Double): Double;

  public
    procedure Init;              // aanroepen vanuit .dpr na CreateNew(nil)
    destructor Destroy; override;
  end;

var
  Form1: TForm1;

implementation

//{$R *.fmx}

{=========================== Jointlijst voor logging (directe index) ===========================}
type
  TJointRow = record
    Name: string;
    Qp  : Integer;   // qpos index
    Dv  : Integer;   // qvel/doF index
  end;

const
  JOINTS: array[0..20] of TJointRow = (
    (Name:'abdomen_z';       Qp: 7; Dv: 6),
    (Name:'abdomen_y';       Qp: 8; Dv: 7),
    (Name:'abdomen_x';       Qp: 9; Dv: 8),
    (Name:'hip_x_right';     Qp:10; Dv: 9),
    (Name:'hip_z_right';     Qp:11; Dv:10),
    (Name:'hip_y_right';     Qp:12; Dv:11),
    (Name:'knee_right';      Qp:13; Dv:12),
    (Name:'ankle_y_right';   Qp:14; Dv:13),
    (Name:'ankle_x_right';   Qp:15; Dv:14),
    (Name:'hip_x_left';      Qp:16; Dv:15),
    (Name:'hip_z_left';      Qp:17; Dv:16),
    (Name:'hip_y_left';      Qp:18; Dv:17),
    (Name:'knee_left';       Qp:19; Dv:18),
    (Name:'ankle_y_left';    Qp:20; Dv:19),
    (Name:'ankle_x_left';    Qp:21; Dv:20),
    (Name:'shoulder1_right'; Qp:22; Dv:21),
    (Name:'shoulder2_right'; Qp:23; Dv:22),
    (Name:'elbow_right';     Qp:24; Dv:23),
    (Name:'shoulder1_left';  Qp:25; Dv:24),
    (Name:'shoulder2_left';  Qp:26; Dv:25),
    (Name:'elbow_left';      Qp:27; Dv:26)
  );

{=========================== Helpers ===========================}

function LSqrt(X : Double) : Double;

begin

  if X > 0 then
    LSqrt := Sqrt(X)
  else
    LSqrt := 0;

end;

function GetJointNameByDof(const dofIdx: Integer): string;
var
  j: Integer;
begin
  for j := 0 to High(JOINTS) do
    if JOINTS[j].Dv = dofIdx then
      Exit(JOINTS[j].Name);
  Result := Format('dof_%d', [dofIdx]); // fallback als hij niet in JOINTS staat
end;

function RandUniform(var s: UInt32; A, B: Double): Double; inline;
begin
  // xorshift32
  s := s xor (s shl 13);
  s := s xor (s shr 17);
  s := s xor (s shl 5);
  Result := A + (B - A) * (s / $FFFFFFFF);
end;

function ClampD(v, lo, hi: Double): Double; inline;
begin
  if v < lo then Exit(lo);
  if v > hi then Exit(hi);
  Result := v;
end;

function TForm1.RandSym(const Amp: Double): Double;
begin
  Result := (Random*2 - 1) * Amp; // [-Amp,+Amp]
end;

function TForm1.Find1Dof(const nameL, nameR: string;
                         out dofL, ndofL, dofR, ndofR: Integer): Boolean;
var ok: Integer;
begin
  Result := False;
  dofL := -1; ndofL := 0; dofR := -1; ndofR := 0;

  ok := MJW_JointNameToDof(H, PAnsiChar(UTF8String(nameL)), dofL, ndofL);
  if (ok <> 0) and (ndofL = 1) then
  begin
    ok := MJW_JointNameToDof(H, PAnsiChar(UTF8String(nameR)), dofR, ndofR);
    if (ok <> 0) and (ndofR = 1) then
      Exit(True);
  end;
end;

function TForm1.FindHipsFallback: Boolean;
begin
  // y → x → z (pak eerste geldige paar)
  if Find1Dof('hip_y_left','hip_y_right',  HipL_FirstDof,HipL_NDof, HipR_FirstDof,HipR_NDof) then Exit(True);
  if Find1Dof('hip_x_left','hip_x_right',  HipL_FirstDof,HipL_NDof, HipR_FirstDof,HipR_NDof) then Exit(True);
  if Find1Dof('hip_z_left','hip_z_right',  HipL_FirstDof,HipL_NDof, HipR_FirstDof,HipR_NDof) then Exit(True);
  Result := False;
end;

function TForm1.FindShouldersAndElbows: Boolean;
begin
  // shoulders: 2 namen per zijde (fallback)
  if not Find1Dof('shoulder1_left','shoulder1_right',
                   ShoulderL_Dof,ShoulderL_NDof, ShoulderR_Dof,ShoulderR_NDof) then
  if not Find1Dof('shoulder2_left','shoulder2_right',
                   ShoulderL_Dof,ShoulderL_NDof, ShoulderR_Dof,ShoulderR_NDof) then
  begin
    ShoulderL_Dof := -1; ShoulderR_Dof := -1;
  end;

  // elbows
  if not Find1Dof('elbow_left','elbow_right',
                   ElbowL_Dof,ElbowL_NDof, ElbowR_Dof,ElbowR_NDof) then
  begin
    ElbowL_Dof := -1; ElbowR_Dof := -1;
  end;

  Result := (ShoulderL_Dof>=0) or (ShoulderR_Dof>=0) or
            (ElbowL_Dof>=0) or (ElbowR_Dof>=0);
end;

function TForm1.MapDof(const nm: AnsiString): Integer;
var dof, nd: Integer;
begin
  Result := -1;
  if MJW_JointNameToDof(H, PAnsiChar(UTF8String(nm)), dof, nd) <> 0 then
    if nd = 1 then
      Result := dof; // 1-DOF → DOF-index direct voor ApplyDof
end;

procedure TForm1.MapExtraDOFs;
begin
  // abdomen
  AbdZ_Dv := MapDof('abdomen_z');
  AbdY_Dv := MapDof('abdomen_y');
  AbdX_Dv := MapDof('abdomen_x');

  // knees
  KneeL_Dv := MapDof('knee_left');   if KneeL_Dv < 0 then KneeL_Dv := MapDof('knee_y_left');
  KneeR_Dv := MapDof('knee_right');  if KneeR_Dv < 0 then KneeR_Dv := MapDof('knee_y_right');

  // ankles
  AnkleL_Dv := MapDof('ankle_left');  if AnkleL_Dv < 0 then AnkleL_Dv := MapDof('ankle_y_left');
  AnkleR_Dv := MapDof('ankle_right'); if AnkleR_Dv < 0 then AnkleR_Dv := MapDof('ankle_y_right');

  // wrists (optioneel)
  WristL_Dv := MapDof('wrist_left');
  WristR_Dv := MapDof('wrist_right');
end;

procedure TForm1.BuildControlledDofs;
  procedure AddDof(const dv: Integer);
  var n: Integer;
  begin
    if dv < 0 then Exit;
    n := Length(FDofs);
    SetLength(FDofs, n+1);
    FDofs[n] := dv;
  end;
begin
  SetLength(FDofs, 0);
  // Volgorde bepaalt NN-output en teacher-order:
  // Armen + ellebogen
  AddDof(ShoulderL_Dof);
  AddDof(ShoulderR_Dof);
  AddDof(ElbowL_Dof);
  AddDof(ElbowR_Dof);
  // Heupen (eerste DOF)
  AddDof(HipL_FirstDof);
  AddDof(HipR_FirstDof);
  // Abdomen
  AddDof(AbdZ_Dv);
  AddDof(AbdY_Dv);
  AddDof(AbdX_Dv);
  // Knieën
  AddDof(KneeL_Dv);
  AddDof(KneeR_Dv);
  // Enkels
  AddDof(AnkleL_Dv);
  AddDof(AnkleR_Dv);
  // Polsen (optioneel)
  AddDof(WristL_Dv);
  AddDof(WristR_Dv);
end;

{=========================== Lifecycle ===========================}

procedure TForm1.Init;
var
  modelPath: string;
  seed: UInt32;
begin
  Caption  := 'MuJoCo – Teach/Predict EBP + Normalized Senses';
  Width    := 900;
  Height   := 230;
  Position := TFormPosition.poScreenCenter;

  // model
  modelPath := TPath.Combine(TPath.GetDirectoryName(ParamStr(0)), 'model\humanoid.xml');
  if not TFile.Exists(modelPath) then
    modelPath := TPath.Combine(TPath.GetDirectoryName(ParamStr(0)), 'model\car.xml');

  StartSim(modelPath);

  // HIPS/ARMS mapping
  if not FindHipsFallback then ShowMessage('Geen hips gevonden');
  if not FindShouldersAndElbows then ShowMessage('Geen shoulders/elbows gevonden');

  // Map extra DOFs
  MapExtraDOFs;

  // Bouw de lijst van aangestuurde DOFs (volgorde is belangrijk)
  BuildControlledDofs;

  // defaults (zoals je had)
  Randomize;
  KickActive     := False;
  KickStartMS    := 0;
  KickDurationMS := 260;
  KickIntervalMS := 280;
  KickTauL       := 0;
  KickTauR       := 0;

  ArmAmpNm   := 220.0;
  ArmFreqHz  := 0.8;
  ElbowGain  := 0.55;
  PhaseR     := Pi;

  // --- Normalisatie parameters ---
  // V50: snelheid (rad/s) waar vel_unit = 0.5
  FVelV50 := 5.0;            // tune: 3..8 afhankelijk van je model
  FVelK   := Ln(2.0) / FVelV50;

  // --- NN setup ---
  // per DOF 3 features (angle_unit, vel_unit, vel_sign) + 4 tijdfeatures
  FInSize   := Length(FDofs) * 3 + 4;
  FHidden   := 64;
  FOutSize  := Length(FDofs) * 2; // *2 : out = pos & neg
  FLR       := {1e-3} {0.04}LearningRate;
  FOutScale := 300.0;        // Nm-orde van grootte van je patronen

  seed := 1337;
  InitNetwork(FInSize, FHidden, FOutSize, seed);

  // Modus
  FMode := moTeach;
  FTrainOnlyInTeach := True;
  FSpaceWasDown := False;

  // timer
  Timer := TTimer.Create(Self);
  Timer.Interval := 16;
  Timer.OnTimer  := OnTick;
  Timer.Enabled  := True;
end;

destructor TForm1.Destroy;
begin
  if Assigned(Timer) then Timer.Enabled := False;
  if H <> nil then MJW_Destroy(H);
  inherited;
end;

procedure TForm1.StartSim(const ModelPath: string);
var
  ansi: AnsiString;
begin
  ansi := AnsiString(ModelPath);
  H := MJW_Create(PAnsiChar(ansi), 1200, 900);
  if H = nil then
    raise Exception.Create('MJW_Create failed: ' + string(MJW_GetLastError));

  MJW_SetMouseCamParams(H, -1, -1, {0.02}0.05);

  MJW_EnableMouseCam(H, 1);

  MJW_SetTimestep(H, 1/240.0);
end;

{=========================== Observaties/Teacher/Apply ==============================}

function TForm1.WrapPi(a: Double): Double; {inline}
begin
  // stabiele wrap naar [-pi, pi]
  Result := ArcTan2(Sin(a), Cos(a));
end;

function TForm1.AngleRadToUnit(a: Double): Double; {inline}
var w: Double;
begin
  w := WrapPi(a);
  Result := (w + Pi) / (2*Pi); // 0..1
end;

function TForm1.VelToUnitAbs(v, k: Double): Double; {inline}
begin
  // 0..inf -> 0..1
  Result := 1.0 - Exp(-Max(0.0, v) * k);
end;

function TForm1.DvToQp(const dv: Integer): Integer; {inline}
var j: Integer;
begin
  for j := 0 to High(JOINTS) do
    if JOINTS[j].Dv = dv then
      Exit(JOINTS[j].Qp);
  Result := -1;
end;

procedure TForm1.ReadObs(out Obs: TArray<Double>);
var
  i, n, qp, base: Integer;
  t, fastF, slowF: Double;
  v, a: Double;

  k : Integer;

  nm: string;
  barLen: Integer;

//const MinAngleUnit = 0.12;

begin
  n := Length(FDofs);

  Writeln(n, '=n');

  SetLength(Obs, FInSize);

  Write(#27'[2J'#27'[H');

  // per DOF: angle_unit, vel_unit, vel_sign
  for i := 0 to n-1 do
  begin
    v := MJW_ReadQvel(H, FDofs[i]);

    qp := DvToQp(FDofs[i]);
    if qp >= 0 then
      a := MJW_ReadQpos(H, qp)
    else
      a := 0.0;

    base := i*3;
    Obs[base + 0] := AngleRadToUnit(a);               // 0..1
    Obs[base + 1] := VelToUnitAbs(Abs(v), FVelK);     // 0..1
    Obs[base + 2] := Sign(v);                         // -1,0,1

(*//    Writeln(Obs[base + 0], ' | ', Obs[base + 1]);

    Write(i : 2, ' -- ');

    for k := 0 to Round(Obs[base + 0]*25) do
      Write(#$2588);

    Writeln; *)

(*  if i < n {DIV 2} then
  begin

  nm := GetJointNameByDof(FDofs[i]);
  barLen := Round(TanH(Obs[base + 0]*{1.9}1.8) * 25);  // angle_unit → lengte 0..25

  Write(Format('%2d %-18s | ', [i, nm]));
  for k := 1 to barLen do
    Write(#$2588);  // █ (Unicode)
  Writeln(Format('  ang=%5.2f  vel=%5.2f  sgn=%2.0f',
                 [Obs[base+0], Obs[base+1], Obs[base+2]]));   // Visualize temp. off

  end; *)

  end;

  // tijdfeatures (helpen ritmes leren)
  t := MJW_GetSimTime(H);
  slowF := ArmFreqHz; // ~0.8 Hz (schouders/elbows)
  fastF := 1.80;      // enkels/knieën ritme

  Obs[n*3 + 0] := Sin(2*Pi*slowF*t);
  Obs[n*3 + 1] := Cos(2*Pi*slowF*t);
  Obs[n*3 + 2] := Sin(2*Pi*fastF*t);
  Obs[n*3 + 3] := Cos(2*Pi*fastF*t);
end;

procedure TForm1.ComputeTeacher(const t: Double; out TauTeacher: TArray<Double>);
var
  nowMS : UInt64;
  w     : Double;
  tauSL,tauSR,tauEL,tauER: Double;
  AmpNm : Double;
  i, idx: Integer;

  SinMuscle : Double;

begin
  SetLength(TauTeacher, Length(FDofs)*2);

  // --------- ARMS (schouder + elleboog sinus) ----------
  w := 2*Pi*ArmFreqHz;
(*  tauSL := ArmAmpNm * Sin(w * t + 0.0);
  tauSR := ArmAmpNm * Sin(w * t + PhaseR);
  tauEL := ElbowGain * tauSL;
  tauER := ElbowGain * tauSR;

  // --------- HIPS (glute random kick) ----------
  nowMS := GetTickCount64;
  if (not KickActive) and ((KickStartMS=0) or (nowMS - KickStartMS >= UInt64(KickIntervalMS))) then
  begin
    AmpNm    := 250.0;
    KickTauL := RandSym(AmpNm);
    KickTauR := RandSym(AmpNm);
    KickStartMS := nowMS;
    KickActive  := True;
  end;
  if KickActive and (nowMS - KickStartMS >= UInt64(KickDurationMS)) then
  begin
    KickActive := False;
    KickTauL := 0; KickTauR := 0;
  end;     *)

  SinMuscle := Sin(w * t + 0.0)*0.5+0.5;

  // ===== Teacher vector opbouwen in exact dezelfde volgorde als FDofs =====
  idx := 0;

  // Armen + ellebogen
  if ShoulderL_Dof>=0 then begin TauTeacher[idx] := {SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); TauTeacher[idx] := 1-{SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); end;
  if ShoulderR_Dof>=0 then begin TauTeacher[idx] := {SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); TauTeacher[idx] := 1-{SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); end;
  if ElbowL_Dof>=0    then begin TauTeacher[idx] := {SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); TauTeacher[idx] := 1-{SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); end;
  if ElbowR_Dof>=0    then begin TauTeacher[idx] := {SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); TauTeacher[idx] := 1-{SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); end;

  // Heupen
  if HipL_FirstDof>=0 then begin TauTeacher[idx] := {SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); TauTeacher[idx] := 1-{SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); end;
  if HipR_FirstDof>=0 then begin TauTeacher[idx] := {SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); TauTeacher[idx] := 1-{SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); end;

  // EXTRA: abdomen (verschillende fases/frequenties zoals voorheen)
  if AbdZ_Dv >= 0 then begin TauTeacher[idx] := {SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); TauTeacher[idx] := 1-{SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); end;
  if AbdY_Dv >= 0 then begin TauTeacher[idx] := {SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); TauTeacher[idx] := 1-{SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); end;
  if AbdX_Dv >= 0 then begin TauTeacher[idx] := {SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); TauTeacher[idx] := 1-{SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); end;

  // Knieën – sterk & sneller; L/R tegenfase
  if KneeL_Dv >= 0 then begin TauTeacher[idx] := {SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); TauTeacher[idx] := 1-{SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); end;
  if KneeR_Dv >= 0 then begin TauTeacher[idx] := {SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); TauTeacher[idx] := 1-{SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); end;

  // Enkels – lichter; nog sneller; L/R tegenfase
  if AnkleL_Dv >= 0 then begin TauTeacher[idx] := {SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); TauTeacher[idx] := 1-{SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); end;
  if AnkleR_Dv >= 0 then begin TauTeacher[idx] := {SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); TauTeacher[idx] := 1-{SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); end;

  // Polsen
//  if WristL_Dv >= 0 then begin TauTeacher[idx] := {SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); TauTeacher[idx] := 1-{SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); end;
//  if WristR_Dv >= 0 then begin TauTeacher[idx] := {SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); TauTeacher[idx] := 1-{SinMuscle}ObsOut[(idx DIV 2)*3]; Inc(idx); end;

  // Safety: als er gaten waren door ontbrekende DOFs, vul rest met nul:
  for i := idx to High(TauTeacher) do
    TauTeacher[i] := 0.0;
end;

procedure TForm1.ApplyForcesVector(const Tau: TArray<Double>);
var
  i, n: Integer;

begin
  n := {Min(Length(Tau), Length(FDofs))}Length({Tau}FDofs);
  MJW_ClearApplied(H);
  for i := {0}{18}0 to n-1 do
  begin

    RanTrace[i*2] := RanTrace[i*2]*{0.935}{0.97}(1-TraceSpd) + {0.065}{0.03}TraceSpd*Random;
    RanTrace[i*2+1] := RanTrace[i*2+1]*{0.935}{0.97}(1-TraceSpd) + {0.065}{0.03}TraceSpd*Random;

//    RanTrace[i*2] := Sin(Cyc*2*Pi*0.15)*0.5+0.5;
//    RanTrace[i*2+1] := Sin(Cyc*2*Pi*0.27)*0.5+0.5;

    TauTrace[i*2] := TauTrace[i*2]*{0.82}{0.997}0.991 + {0.18}{0.003}0.009*(Tau[i*2]);
    TauTrace[i*2+1] := TauTrace[i*2+1]*{0.82}0.997 + {0.18}0.003*(Tau[i*2+1]);   // --> important : tweak fatigue trace!!

    if (i >= 12) AND (i <= 17) then
    begin

    MJW_ApplyDof(H, FDofs[i], {800}AbdMul*MaxForce*(Sqr(Tau[i*2])-Sqr(TauTrace[i*2]))*RanTrace[i*2]);
    MJW_ApplyDof(H, FDofs[i], {800}AbdMul*MaxForce*-(Sqr(Tau[i*2+1])-Sqr(TauTrace[i*2]))*RanTrace[i*2+1]);

    end
    else
    if (i >= 8) AND (i <= 11) then
    begin

    MJW_ApplyDof(H, FDofs[i], {800}HipMul*MaxForce*(Sqr(Tau[i*2])-Sqr(TauTrace[i*2]))*RanTrace[i*2]);
    MJW_ApplyDof(H, FDofs[i], {800}HipMul*MaxForce*-(Sqr(Tau[i*2+1])-Sqr(TauTrace[i*2+1]))*RanTrace[i*2+1]);

    end
    else
    if (i >= 18) AND (i <= 21) then
    begin

    MJW_ApplyDof(H, FDofs[i], {800}LegMul*MaxForce*(Sqr(Tau[i*2])-Sqr(TauTrace[i*2]))*RanTrace[i*2]);
    MJW_ApplyDof(H, FDofs[i], {800}LegMul*MaxForce*-(Sqr(Tau[i*2+1])-Sqr(TauTrace[i*2+1]))*RanTrace[i*2+1]);

    end
    else
    begin

    MJW_ApplyDof(H, FDofs[i], {800}MaxForce*(Sqr(Tau[i*2])-Sqr(TauTrace[i*2]))*RanTrace[i*2]);
    MJW_ApplyDof(H, FDofs[i], {800}MaxForce*-(Sqr(Tau[i*2+1])-Sqr(TauTrace[i*2+1]))*RanTrace[i*2+1]);

    end;

    // -

  end;

  Inc(Cyc);

end;

{=========================== NN =====================}

function TForm1.Idx(OutIdx, InIdx, InSize: Integer): Integer;
begin
  Result := OutIdx * InSize + InIdx;
end;

class function TForm1.Tanh_(v: Double): Double;
begin
  Result := System.Math.Tanh(v);
end;

class function TForm1.DTanh_(post: Double): Double;
begin
  // post = tanh(z)
  Result := 1.0 - post * post;
end;

procedure TForm1.InitNetwork(InSize, Hidden, OutSize: Integer; Seed: UInt32);
var
  i: Integer;
  lim1, lim2: Double;
begin
  SetLength(W1, Hidden * InSize);
  SetLength(B1, Hidden);
  SetLength(W2, OutSize * Hidden);
  SetLength(B2, OutSize);
  SetLength(X, InSize);
  SetLength(Self.Hidden, Hidden);
  SetLength(Y, OutSize);

  SetLength(RanTrace, OutSize);

  SetLength(TauTrace, OutSize);

  SetLength(OutW, OutSize); // important.

  for i := 0 to InputDelayLen do
    SetLength(PrevObsOut[i], (FOutSize DIV 2)*3+4);

  SetLength(ObsOut, (FOutSize DIV 2)*3+4);

  lim1 := Sqrt(6.0 / (InSize + Hidden));
  lim2 := Sqrt(6.0 / (Hidden + OutSize));

  for i := 0 to High(W1) do W1[i] := RandUniform(Seed, -lim1, lim1);
  for i := 0 to High(B1) do B1[i] := 0.0;

  for i := 0 to High(W2) do W2[i] := RandUniform(Seed, -lim2, lim2);
  for i := 0 to High(B2) do B2[i] := 0.0;
end;

procedure TForm1.PropagateForward(const Input: TArray<Double>; out Output: TArray<Double>);
var
  o, i: Integer;
  z: Double;
begin
  if Length(Input) <> FInSize then
    raise Exception.Create('PropagateForward: input size mismatch');

  X := Copy(Input);

  // Hidden = tanh(W1*X + B1)
  for o := 0 to FHidden - 1 do
  begin
    z := B1[o];
    for i := 0 to FInSize - 1 do
      z := z + W1[Idx(o, i, FInSize)] * X[i];
    Hidden[o] := Tanh_(z);
  end;

  // Raw output Y = tanh(W2*Hidden + B2) in [-1,1]
  SetLength(Output, FOutSize);
  for o := 0 to FOutSize - 1 do
  begin
    z := B2[o];
    for i := 0 to FHidden - 1 do
      z := z + W2[Idx(o, i, FHidden)] * Hidden[i];
    Y[o] := Tanh_(z); // raw

    //RanTrace[o] := RanTrace[o]*0.935 + 0.065*Random;

    Output[o] := (*ClampD(Y[o] * FOutScale, -1e9, 1e9)*{(Sin(SinCycle*2*Pi*0.0055)*2+1)}*)Y[o]{*RanTrace[o]} {Rand fact later.} {*2.5}{Random*2000}; // scaled Nm for actuation

  end;

  Inc(SinCycle);

end;

function TForm1.PropagateBack(const Input, TargetNm: TArray<Double>; LR: Double): Double;
var
  o, i: Integer;
  dZ2, dB2, dB1: TArray<Double>;
  dW2, dW1: TArray<Double>;
  mse, errNm, sum: Double;
  outScaled: TArray<Double>;
begin
  if (Length(Input) <> FInSize) or (Length(TargetNm) <> FOutSize) then
    raise Exception.Create('PropagateBack: size mismatch');

  // Forward (fills X,Hidden,Y), and get scaled output (= Nm) for loss
  PropagateForward(Input, outScaled);

  // MSE on scaled torques
  mse := 0.0;
  for o := 0 to FOutSize - 1 do
  begin
    errNm := {0.4*Random +} outScaled[o] - TargetNm[o];  // error in Nm  -- > * Random : exploration noise  --> placed @ next.
    mse := mse + Sqr(errNm);
  end;
  Result := mse / Max(1, FOutSize);

  // Backprop:
  // outScaled = Y * FOutScale, with Y = tanh(z). So:
  // dLoss/dY = (outScaled - Target) * d(outScaled)/dY = errNm * FOutScale
  // dY/dz = tanh'(z) = 1 - Y^2  (with Y = tanh(z))
  // => dLoss/dz = errNm * FOutScale * (1 - Y^2)
  SetLength(dZ2, FOutSize);
  for o := 0 to FOutSize - 1 do
    dZ2[o] := (outScaled[o] - TargetNm[o]*Random) {* FOutScale * DTanh_(Y[o])};

  // Grad W2 = outer(dZ2, Hidden), Grad B2 = dZ2
  SetLength(dW2, FOutSize * FHidden);
  SetLength(dB2, FOutSize);
  for o := 0 to FOutSize - 1 do
  begin
    dB2[o] := dZ2[o];
    for i := 0 to FHidden - 1 do
      dW2[Idx(o, i, FHidden)] := dZ2[o] * Hidden[i];
  end;

  // Backprop naar hidden: dHidden = W2^T * dZ2 .* tanh'(Hidden)
  SetLength(dB1, FHidden);
  for i := 0 to FHidden - 1 do
  begin
    sum := 0.0;
    for o := 0 to FOutSize - 1 do
      sum := sum + W2[Idx(o, i, FHidden)] * dZ2[o];
    dB1[i] := sum * DTanh_(Hidden[i]);
  end;

  // Grad W1 = outer(dB1, X)
  SetLength(dW1, FHidden * FInSize);
  for o := 0 to FHidden - 1 do
    for i := 0 to FInSize - 1 do
      dW1[Idx(o, i, FInSize)] := dB1[o] * X[i];

  // SGD updates
  for o := 0 to FOutSize - 1 do
  begin
    B2[o] := B2[o] - LR * dB2[o];
    for i := 0 to FHidden - 1 do
      W2[Idx(o, i, FHidden)] := W2[Idx(o, i, FHidden)] - LR * dW2[Idx(o, i, FHidden)];
  end;

  for o := 0 to FHidden - 1 do
  begin
    B1[o] := B1[o] - LR * dB1[o];
    for i := 0 to FInSize - 1 do
      W1[Idx(o, i, FInSize)] := W1[Idx(o, i, FInSize)] - LR * dW1[Idx(o, i, FInSize)];
  end;
end;

{=========================== OnTick ==============================}

procedure TForm1.OnTick(Sender: TObject);
var
  t     : Double;
  SubSteps: Integer;
  s,k  : Integer;

  // teacher/nn
  Obs, TauTeacher, TauNN, TauOut: TArray<Double>;
  loss: Double;

  // logging locals
  q,v,qn,vn: Double;

  // space polling
  spaceDown: ShortInt;

  i, j, n : Integer;

  nm: string;
  barLen: Integer;

begin
  if H = nil then Exit;

  // --- Spacebar polling (debounce) ---
  spaceDown := ShortInt(GetAsyncKeyState(VK_SPACE) shr 8); // <0 betekent ingedrukt
  if (spaceDown < 0) and (not FSpaceWasDown) then
  begin
    if FMode = moTeach then FMode := moPredict else FMode := moTeach;
    WriteLn('MODE -> ' + IfThen(FMode=moTeach,'Teach','Predict'));
  end;
  FSpaceWasDown := (spaceDown < 0);

  t := MJW_GetSimTime(H);

//  // --- Teacher (jouw patronen) ---
//  ComputeTeacher(t, TauTeacher);

  // --- Observaties ---
  ReadObs(Obs);

  PreObsOut := Copy(Obs);

//  for i := 0 to FOutSize - 1 do

  for i := InputDelayLen downto 1 do
    PrevObsOut[i] := Copy(PrevObsOut[i-1]);

  PrevObsOut[0] := Copy(PreObsOut);

  //SetLength(ObsOut, FOutSize);

//.  Writeln('FOutSize = ', FOutSize);

//  SetLength(ObsOut, (FOutSize DIV 2)*3+4);

  for i := 0 to (FOutSize DIV 2)*3 - 1 do
    ObsOut[i] := (PrevObsOut[0,i]-PrevObsOut[Random(InputDelayLen+1-9)+9,i])*0.5+0.5;


//  ObsOut := Copy(Obs);

  // --- Teacher (jouw patronen) ---
  ComputeTeacher(t, TauTeacher);

  // --- Train / Predict ---
  case FMode of
    moTeach:
      begin
        loss := PropagateBack(Obs, TauTeacher, FLR); // train
//        TauOut := Copy(TauTeacher);                  // stuur teacher uit

        PropagateForward(Obs, TauNN);                 // pure NN-uitgang
        TauOut := Copy(TauNN);

      end;
    moPredict:
      begin
        if not FTrainOnlyInTeach then
          loss := PropagateBack(Obs, TauTeacher, FLR)  // optional continual learning
        else
          loss := 0.0;
        PropagateForward(Obs, TauNN);                 // pure NN-uitgang
        TauOut := Copy(TauNN);
      end;
  end;

  // --------- Substeps ----------
  SubSteps := 4;   // 3..4 is mooi; hoger = meer CPU
  for s := 1 to SubSteps do
  begin
    ApplyForcesVector(TauOut);
    // physics step
    MJW_Step(H);
  end;

  // --

//  Writeln;
//  Writeln;

  if FMode = MoTeach then
    Writeln(' -- Teach mode --')
  else
    Writeln(' -- Predict mode --');

  if ConsoleVizOn then
  begin

  Writeln;

  n := {Min(Length(TauOut), Length(FDofs))}Length(TauOut);

//  Writeln(Length(TauOut), '=tau');

  for i := 0 to n-1 do
  begin

  if i < n{ DIV 2} then
  begin

  nm := GetJointNameByDof(FDofs[i DIV 2]);
  barLen := Round(TauOut[i]{TauTeacher[i]}{PreObsOut[(i DIV 2)*3]}{ObsOut[(i DIV 2)*3]} {*100 * 0.25} *25);  // angle_unit → lengte 0..25

  Write(Format('%2d %-18s | ', [i, nm]));
  for j := 1 to barLen do
    Write(#$2588);  // █ (Unicode)
//  Writeln(Format('  ang=%5.2f  vel=%5.2f  sgn=%2.0f',
//                 [Obs[base+0], Obs[base+1], Obs[base+2]]));

  Writeln;
  end;
  end;  // <-- visualize.

  end;

  // --

  // render + events
  MJW_Render(H);
  MJW_PollEvents(H);

  Inc(FrameCount);


//  Winapi.Windows.System('cls');

(*  Write(#27'[2J'#27'[H');

  // ===== Console: ALLE JOINTS – logging =====
  begin
    WriteLn(Format('==== JOINTS @ t=%.3f (n=%d)  MODE=%s  loss=%.6f ====',
      [MJW_GetSimTime(H), Length(JOINTS),
       IfThen(FMode=moTeach,'Teach','Predict'), loss]));
    for k := 0 to High(JOINTS) do
    begin
      // DIRECT INDEX
      q := MJW_ReadQpos(H, JOINTS[k].Qp);
      v := MJW_ReadQvel(H, JOINTS[k].Dv);
      WriteLn(Format('direct idx: %-18s q=%s  v=%s',
        [ JOINTS[k].Name, FloatToStr(q), FloatToStr(v) ]));

      // NAME ROUTE
{      qn := MJW_Joint1D_Angle(H, PAnsiChar(UTF8String(JOINTS[k].Name)));
      vn := MJW_Joint1D_Vel  (H, PAnsiChar(UTF8String(JOINTS[k].Name)));
      WriteLn(Format('name route: %-12s q=%s  v=%s',
        [ JOINTS[k].Name, FloatToStr(qn), FloatToStr(vn) ])); }

//      WriteLn;
    end;
    WriteLn('======================================');
  end; *)

  if MJW_ShouldClose(H) <> 0 then Close;
end;

end.

