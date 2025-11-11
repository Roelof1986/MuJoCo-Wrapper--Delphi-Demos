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
