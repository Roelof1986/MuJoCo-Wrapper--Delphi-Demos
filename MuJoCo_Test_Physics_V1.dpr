program MuJoCo_Test_Physics_V1;

uses
  System.StartUpCopy,
  FMX.Forms,
  Unit1 in 'Unit1.pas';

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
