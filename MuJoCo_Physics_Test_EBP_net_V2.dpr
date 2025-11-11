{**************************************************************************
  Project : MuJoCo EBP Demo (Delphi/FMX)
  File    : MuJoCo_Physics_Test_EBP_net_V2.dpr
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

program MuJoCo_Physics_Test_EBP_net_V2;

uses
  System.StartUpCopy,
  FMX.Forms,
  Unit1_EBP_net in 'Unit1_EBP_net.pas';

{$R *.res}

begin
  Application.Initialize;

  // Maak een resource-loze form (géén .fmx nodig)
  Form1 := TForm1.CreateNew(nil);

  // Init alle logica (eigenschappen/venster + MuJoCo-start)
  Form1.Init;

  Application.MainForm := Form1;
  Application.Run;
end.
