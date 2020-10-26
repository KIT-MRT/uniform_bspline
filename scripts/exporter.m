(*
Mathematica package with helper functions to export data from mathematica to a text file.
*)
BeginPackage["Exporter`"]

matToString::usage = "Converts a matrix to a string with the given separator."

Begin["`Private`"]

matToString[mat_, sep_] := Module[{},
    StringRiffle[(ToString[CForm[N[#, 20]]])&/@mat, sep]
]

End[]
EndPackage[]
