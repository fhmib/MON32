#$language = "VBScript"
#$interface = "1.0"

crt.Screen.Synchronous = True

' This automatically generated script may need to be
' edited in order to work correctly.

Sub Main
	do
		crt.Screen.Send chr(13) & "txsw write 32" & chr(13)
		crt.Screen.WaitForString "Returned status from module is 0 (= 0)", 2
		crt.Screen.Send chr(13) & "txsw read" & chr(13)
		crt.Screen.WaitForString "TX switch channel is 32", 2
		' crt.Sleep 2000
		crt.Screen.Send chr(13) & "txsw write 1" & chr(13)
		crt.Screen.WaitForString "Returned status from module is 0 (= 0)", 2
		crt.Screen.Send chr(13) & "txsw read" & chr(13)
		crt.Screen.WaitForString "TX switch channel is 1", 2
	loop
End Sub
