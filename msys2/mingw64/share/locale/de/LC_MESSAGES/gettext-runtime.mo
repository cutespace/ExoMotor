��    &      L  5   |      P  8   Q  B   �  A   �  6     H   F  I   �  F   �  9      7   Z  6   �  M   �  L     O   d  H   �  {   �     y  �   �  e   `  :   �    	  �  
  �  �     �     �  <   �  1   �  &   "     I  "   X  9   {  I   �  �   �     �     �     �     �     �  �  �  E   �  C   3  @   w  <   �  E   �  G   ;  J   �  @   �  =     E   M  Q   �  q   �  �   W  E   �     4     �  �   �  {   �  I   '  @  q  '  �  �  �     �     �  e   �  ,     *   G     r  #   �  D   �  V   �  �   A                 9      J      Z               !          #                                                      %                                        
                  $      	   &                 "                  -E                        (ignored for compatibility)
   -V, --version               output version information and exit
   -V, --version             display version information and exit
   -c, --context=CONTEXT     specify context for MSGID
   -d, --domain=TEXTDOMAIN   retrieve translated message from TEXTDOMAIN
   -d, --domain=TEXTDOMAIN   retrieve translated messages from TEXTDOMAIN
   -e                        enable expansion of some escape sequences
   -h, --help                  display this help and exit
   -h, --help                display this help and exit
   -n                        suppress trailing newline
   -v, --variables             output the variables occurring in SHELL-FORMAT
   COUNT                     choose singular/plural form based on this value
   MSGID MSGID-PLURAL        translate MSGID (singular) / MSGID-PLURAL (plural)
   [TEXTDOMAIN]              retrieve translated message from TEXTDOMAIN
   [TEXTDOMAIN] MSGID        retrieve translated message corresponding
                            to MSGID from TEXTDOMAIN
 Bruno Haible Copyright (C) %s Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <%s>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.
 Display native language translation of a textual message whose grammatical
form depends on a number.
 Display native language translation of a textual message.
 If the TEXTDOMAIN parameter is not given, the domain is determined from the
environment variable TEXTDOMAIN.  If the message catalog is not found in the
regular directory, another location can be specified with the environment
variable TEXTDOMAINDIR.
Standard search directory: %s
 If the TEXTDOMAIN parameter is not given, the domain is determined from the
environment variable TEXTDOMAIN.  If the message catalog is not found in the
regular directory, another location can be specified with the environment
variable TEXTDOMAINDIR.
When used with the -s option the program behaves like the 'echo' command.
But it does not simply copy its arguments to stdout.  Instead those messages
found in the selected catalog are translated.
Standard search directory: %s
 In normal operation mode, standard input is copied to standard output,
with references to environment variables of the form $VARIABLE or ${VARIABLE}
being replaced with the corresponding values.  If a SHELL-FORMAT is given,
only those environment variables that are referenced in SHELL-FORMAT are
substituted; otherwise all environment variables references occurring in
standard input are substituted.
 Informative output:
 Operation mode:
 Report bugs in the bug tracker at <%s>
or by email to <%s>.
 Substitutes the values of environment variables.
 Try '%s --help' for more information.
 Ulrich Drepper Usage: %s [OPTION] [SHELL-FORMAT]
 Usage: %s [OPTION] [TEXTDOMAIN] MSGID MSGID-PLURAL COUNT
 Usage: %s [OPTION] [[TEXTDOMAIN] MSGID]
or:    %s [OPTION] -s [MSGID]...
 When --variables is used, standard input is ignored, and the output consists
of the environment variables that are referenced in SHELL-FORMAT, one per line.
 Written by %s.
 error while reading "%s" missing arguments standard input too many arguments Project-Id-Version: gettext-runtime 0.20.2
Report-Msgid-Bugs-To: bug-gettext@gnu.org
PO-Revision-Date: 2020-04-18 23:24+0200
Last-Translator: Mario Blättermann <mario.blaettermann@gmail.com>
Language-Team: German <translation-team-de@lists.sourceforge.net>
Language: de
MIME-Version: 1.0
Content-Type: text/plain; charset=UTF-8
Content-Transfer-Encoding: 8bit
X-Bugs: Report translation errors to the Language-Team address.
Plural-Forms: nplurals=2; plural=(n != 1);
X-Generator: Lokalize 19.12.3
   -E                        (zwecks Kompatibilität; wird ignoriert)
   -V, --version                Versionsnummer anzeigen und beenden
   -V, --version             Versionsnummer anzeigen und beenden
   -c, --context=KONTEXT     Kontext für SCHLÜSSEL angeben
   -d, --domain=TEXTDOMAIN   übersetzte Meldung aus TEXTDOMAIN holen
   -d, --domain=TEXTDOMAIN   Übersetzte Meldungen aus TEXTDOMAIN holen
   -e                        Expansion einiger Escape-Sequenzen aktivieren
   -h, --help                   diese Hilfe anzeigen und beenden
   -h, --help                diese Hilfe anzeigen und beenden
   -n                        angehängten Zeilenumbruch unterdrücken
   -v, --variables              Variablen ausgeben, die im SHELL-FORMAT vorkommen
   ZAHL                      Singular-/Pluralform basierend auf
                              diesem Wert wählen
   SCHLÜSSEL SCHLÜSSEL-PLURAL
                            SCHLÜSSEL (Singular) / SCHLÜSSEL-PLURAL
                            (Plural) übersetzen
   [TEXTDOMAIN]              übersetzte Meldung aus TEXTDOMAIN holen
   [TEXTDOMAIN] SCHLÜSSEL    Übersetzte Meldung entsprechend des
                            SCHLÜSSELs aus TEXTDOMAIN holen
 Bruno Haible Copyright © %s Free Software Foundation, Inc.
Lizenz GPLv3: GNU GPL Version 3 oder neuer <%s>
Dies ist freie Software; es steht Ihnen frei, sie zu verändern und
weiterzugeben.
Es gibt KEINE GARANTIE, soweit als vom Gesetz erlaubt.
 Übersetzung einer textuellen Meldung, deren grammatische Form von einer Zahl
abhängt, in einer nativen Sprache anzeigen.
 Übersetzung einer textuellen Meldung in einer nativen Sprache anzeigen.
 Wenn der Parameter BEREICH nicht angegeben wurde, wird der Bereich durch die
Umgebungsvariable TEXTDOMAIN bestimmt. Wenn der Katalog dieses Bereiches
sich nicht im Standardverzeichnis des Systems befindet, kann durch die
Umgebungsvariable TEXTDOMAINDIR ein anderes Verzeichnis angegeben werden.

Standardverzeichnis: %s
 Wenn der Parameter TEXTBEREICH nicht angegeben wurde, wird der Bereich durch
die Umgebungsvariable TEXTDOMAIN bestimmt. Wenn der Katalog dieses Bereiches
sich nicht im Standardverzeichnis des Systems befindet, kann durch die
Umgebungsvariable TEXTDOMAINDIR ein anderes Verzeichnis angegeben werden.

Wenn die Option »-s« angegeben ist, verhält sich das Programm wie der Befehl
»echo«. Aber anstatt alle Argumente einfach auszugeben, werden diese
übersetzt, falls sie im angegebenen Katalog (Textbereich) vorhanden sind.

Standardverzeichnis: %s
 Im normalen Betriebsmodus wird die Standardeingabe zur Standardausgabe kopiert.
Dabei werden Referenzen auf Umgebungsvariablen in der Formatform $VARIABLE oder
${VARIABLE} durch die entsprechenden Werte ersetzt. Wenn ein SHELL-FORMAT
angegeben ist, werden nur die Umgebungsvariablen, die im SHELL-FORMAT
referenziert, substituiert; andernfalls werden alle Referenzen auf
Umgebungsvariablen bei der Standardausgabe substituiert.
 Informative Ausgabe:
 Art der Verarbeitung:
 Melden Sie Fehler im Fehlererfassungssystem auf <%s>
oder per E-Mail an <%s> (jeweils auf Englisch).
 Werte der Umgebungsvariablen substituieren.
 »%s --help« gibt weitere Informationen.
 Ulrich Drepper Aufruf: %s [OPTION] [SHELL-FORMAT]
 Aufruf: %s [OPTION] [TEXTBEREICH] SCHLÜSSEL SCHLÜSSEL-PLURAL ZAHL
 Aufruf: %s [OPTION] [[TEXTBEREICH] SCHLÜSSEL]
oder:   %s [OPTION] -s [SCHLÜSSEL]…
 Wenn --variables verwendet wird, wird die Standardeingabe ignoriert und die
Standardausgabe wird aus allen Umgebungsvariablen bestehen, die im
SHELL-FORMAT referenziert sind, immer eine pro Zeile.
 Geschrieben von %s.
 Fehler beim Lesen von »%s« Argumente fehlen Standardeingabe zu viele Argumente 