; Piksi Console NSIS Installer script

;--------------------------------

; The name of the installer
Name "Piksi Console"

; The file to write
OutFile "piksi_console_setup.exe"

; The default installation directory
InstallDir "$PROGRAMFILES\Swift Navigation\Piksi Console"

; The text to prompt the user to enter a directory
DirText "This will install Piksi Console on your computer. Choose a directory"

;--------------------------------

; The stuff to install
Section ""

; Set output path to the installation directory.
SetOutPath $INSTDIR

; Put a file there
File /r dist\console\*

; Now create shortcuts
CreateDirectory "$SMPROGRAMS\Swift Navigation"
CreateShortCut "$SMPROGRAMS\Swift Navigation\Piksi Console.lnk" "$INSTDIR\console.exe"
CreateShortCut "$SMPROGRAMS\Swift Navigation\Uninstall.lnk" "$INSTDIR\Uninstall.exe"

; Tell the compiler to write an uninstaller and to look for a "Uninstall" section
WriteUninstaller $INSTDIR\Uninstall.exe

SectionEnd ; end the section

; The uninstall section
Section "Uninstall"

RMDir /r /REBOOTOK "$INSTDIR/.."

; Now remove shortcuts too
Delete "$SMPROGRAMS\Swift Navigation\Piksi Console.lnk"
Delete "$SMPROGRAMS\Swift Navigation\Uninstall.lnk"
RMDir "$SMPROGRAMS\Swift Navigation"

SectionEnd

