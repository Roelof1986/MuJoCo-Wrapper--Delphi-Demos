{**************************************************************************
  Project : MuJoCo Test Physics (Delphi/FMX)
  File    : Unit1.pas
  Author  : Roelof Emmerink
  Year    : 2025
  License : MIT
  SPDX-License-Identifier: MIT

  ------------------------------------------------------------------------
  Copyright (c) 2025 Roelof Emmerink

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  
  ------------------------------------------------------------------------
  Third-Party Components
  ------------------------------------------------------------------------
  This project integrates the MuJoCo physics engine (https://mujoco.org)
  developed by Google DeepMind.

  MuJoCo is licensed under the Apache License 2.0.
  A full copy of that license is provided in `licenses/Apache-2.0.txt`
  and referenced from `THIRD_PARTY_NOTICES.txt` and `NOTICE`.
***************************************************************************}
 
 unit Unit1;

interface

uses
  System.SysUtils, System.Types, System.Classes, System.IOUtils, System.Math,
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
    // abdomen (3 assen), knieën, enkels, polsen
    AbdZ_Dv, AbdY_Dv, AbdX_Dv : Integer;
    KneeL_Dv, KneeR_Dv        : Integer;
    AnkleL_Dv, AnkleR_Dv      : Integer;
    WristL_Dv, WristR_Dv      : Integer;

    // ---------------- helpers ----------------
    function  RandSym(const Amp: Double): Double;
    function  Find1Dof(const nameL, nameR: string;
                       out dofL, ndofL, dofR, ndofR: Integer): Boolean;
    function  FindHipsFallback: Boolean;
    function  FindShouldersAndElbows: Boolean;

    function  MapDof(const nm: AnsiString): Integer;
    procedure MapExtraDOFs;  // abdomen/knees/ankles/wrists

    // sim
    procedure StartSim(const ModelPath: string);
    procedure OnTick(Sender: TObject);

  public
    procedure Init;              // aanroepen vanuit .dpr na CreateNew(nil)
    destructor Destroy; override;
  end;

var
  Form1: TForm1;

implementation

{=========================== Jointlijst voor logging (directe index) ===========================}
// EXACTE namen + indices (qp/dv) zoals in je eigen dump
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

{=========================== Lifecycle ===========================}

procedure TForm1.Init;
var
  modelPath: string;
begin
  Caption  := 'MuJoCo – All joints logging';
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

  // defaults
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

{=========================== OnTick ==============================}

procedure TForm1.OnTick(Sender: TObject);
var
  nowMS : UInt64;
  t, w  : Double;
  tauSL,tauSR,tauEL,tauER: Double;
  AmpNm : Double;
  s, SubSteps: Integer;

  // logging locals
  k,i: Integer;
  q,v,qn,vn: Double;

begin
  if H = nil then Exit;

  nowMS := GetTickCount64;
  t     := MJW_GetSimTime(H);
  w     := 2*Pi*ArmFreqHz;

  // --------- ARMS (schouder + elleboog sinus) ----------
  tauSL := ArmAmpNm * Sin(w * t + 0.0);
  tauSR := ArmAmpNm * Sin(w * t + PhaseR);
  tauEL := ElbowGain * tauSL;
  tauER := ElbowGain * tauSR;

  // --------- HIPS (glute random kick) ----------
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
  end;

  // --------- Substeps ----------
  SubSteps := 4;   // 3..4 is mooi; hoger = meer CPU
  for s := 1 to SubSteps do
  begin
    // basis: wissen en hips/arms bouwen
    MJW_ClearApplied(H);

    // Arms (sinus)
    if ShoulderL_Dof>=0 then MJW_ApplyDof(H, ShoulderL_Dof, tauSL);
    if ShoulderR_Dof>=0 then MJW_ApplyDof(H, ShoulderR_Dof, tauSR);
    if ElbowL_Dof>=0    then MJW_ApplyDof(H, ElbowL_Dof,    tauEL);
    if ElbowR_Dof>=0    then MJW_ApplyDof(H, ElbowR_Dof,    tauER);

    // Hips (random kick) – additief
    if KickActive then
    begin
      if HipL_FirstDof>=0 then MJW_ApplyDof(H, HipL_FirstDof, KickTauL);
      if HipR_FirstDof>=0 then MJW_ApplyDof(H, HipR_FirstDof, KickTauR);
    end;

    // ===== EXTRA ledematen – additief, DIRECT ApplyDof =====
    // ABDOMEN (z,y,x) – verschillende fases
    if AbdZ_Dv >= 0 then MJW_ApplyDof(H, AbdZ_Dv,  120 * Sin( (2*Pi*1.00)*t + 0.0   ));
    if AbdY_Dv >= 0 then MJW_ApplyDof(H, AbdY_Dv,  140 * Sin( (2*Pi*0.90)*t + Pi/2  ));
    if AbdX_Dv >= 0 then MJW_ApplyDof(H, AbdX_Dv,  100 * Sin( (2*Pi*1.20)*t + Pi    ));

    // KNEES – sterk & sneller; L/R tegenfase
    if KneeL_Dv >= 0 then MJW_ApplyDof(H, KneeL_Dv, 260 * Sin( (2*Pi*1.50)*t + 0.0 ));
    if KneeR_Dv >= 0 then MJW_ApplyDof(H, KneeR_Dv, 260 * Sin( (2*Pi*1.50)*t + Pi  ));

    // ANKLES – lichter; nog sneller; L/R tegenfase
    if AnkleL_Dv >= 0 then MJW_ApplyDof(H, AnkleL_Dv, 160 * Sin( (2*Pi*1.80)*t + 0.0 ));
    if AnkleR_Dv >= 0 then MJW_ApplyDof(H, AnkleR_Dv, 160 * Sin( (2*Pi*1.80)*t + Pi  ));

    // WRISTS (optioneel)
    if WristL_Dv >= 0 then MJW_ApplyDof(H, WristL_Dv,  80 * Sin( (2*Pi*2.00)*t + 0.0 ));
    if WristR_Dv >= 0 then MJW_ApplyDof(H, WristR_Dv,  80 * Sin( (2*Pi*2.00)*t + Pi  ));

    // physics step
    MJW_Step(H);
  end;

  // render + events
  MJW_Render(H);
  MJW_PollEvents(H);

  Inc(FrameCount);

  // ===== Console: ALLE JOINTS – EXACT zoals hip_y_left (direct idx + name route) =====
  if (FrameCount mod 10 = 0) then
  begin
    WriteLn(Format('==== JOINTS @ t=%.3f (n=%d) ====', [MJW_GetSimTime(H), Length(JOINTS)]));
    for k := 0 to High(JOINTS) do
    begin
      // DIRECT INDEX (identiek aan je hip_y_left direct idx stijl)
      q := MJW_ReadQpos(H, JOINTS[k].Qp);
      v := MJW_ReadQvel(H, JOINTS[k].Dv);
      WriteLn(Format('direct idx: %-18s q=%s  v=%s',
        [ JOINTS[k].Name, FloatToStr(q), FloatToStr(v) ]));

      // NAME ROUTE (identiek aan je hip_y_left name route)
      qn := MJW_Joint1D_Angle(H, PAnsiChar(UTF8String(JOINTS[k].Name)));
      vn := MJW_Joint1D_Vel  (H, PAnsiChar(UTF8String(JOINTS[k].Name)));
      WriteLn(Format('name route: %-12s q=%s  v=%s',
        [ JOINTS[k].Name, FloatToStr(qn), FloatToStr(vn) ]));

      WriteLn; // lege regel, precies zoals je eerder deed
    end;
    WriteLn('======================================');
  end;

  if MJW_ShouldClose(H) <> 0 then Close;
end;

end.

