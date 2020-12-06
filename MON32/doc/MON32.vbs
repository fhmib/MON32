#$language = "VBScript"
#$interface = "1.0"

crt.Screen.Synchronous = True

' This automatically generated script may need to be
' edited in order to work correctly.

Sub Main
	do
		crt.Screen.Send chr(13) & "txsw write 1 to 1" & chr(13)
		crt.Screen.WaitForString ">", 2
		crt.Sleep 2000
		crt.Screen.Send "txsw read" & chr(13)

		crt.Screen.Send chr(13) & "debug txsw_io 2 to 22" & chr(13)
		crt.Screen.WaitForString ">", 2
		crt.Sleep 2000
		crt.Screen.Send "txsw read" & chr(13)

		crt.Screen.Send chr(13) & "rxsw write 1" & chr(13)
		crt.Screen.WaitForString ">", 2
		crt.Sleep 2000
		crt.Screen.Send "rxsw read" & chr(13)

		crt.Screen.Send chr(13) & "debug rxsw_io 27" & chr(13)
		crt.Screen.WaitForString ">", 2
		crt.Sleep 2000
		crt.Screen.Send "rxsw read" & chr(13)

	loop
End Sub
