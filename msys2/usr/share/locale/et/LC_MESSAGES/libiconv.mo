��    -      �  =   �      �  C   �  9   %  o   _  B   �  m     ?   �  \   �  ;     P   Y  [   �       @   	  N   J  J   �  D   �  d   )  �   �  :   )	     d	     r	     {	  0   �	     �	  5   �	  	   
     
  )   "
  "   L
  1   o
  &   �
  A   �
  ;   
     F  /   V  7   �  3   �  :   �  ;   -  $   i     �     �     �     �  2   �  �    B   �  E     f   U  H   �  k     C   q  W   �  H     <   V  W   �     �  >   �  <   -  ;   j  5   �  M   �  �   *  ,   �     �     �       2        Q  -   b     �     �  7   �  (   �  5   	  )   ?  H   i  >   �     �  ,     4   .  H   c  5   �  7   �  .     !   I  "   k     �     �  5   �             #                                    $         (   -             +                                    	                &      !   '   *   ,                  
                  "          %      )            --byte-subst=FORMATSTRING   substitution for unconvertible bytes
   --help                      display this help and exit
   --unicode-subst=FORMATSTRING
                              substitution for unconvertible Unicode characters
   --version                   output version information and exit
   --widechar-subst=FORMATSTRING
                              substitution for unconvertible wide characters
   -c                          discard unconvertible characters
   -f ENCODING, --from-code=ENCODING
                              the encoding of the input
   -l, --list                  list the supported encodings
   -s, --silent                suppress error messages about conversion problems
   -t ENCODING, --to-code=ENCODING
                              the encoding of the output
 %s %s argument: A format directive with a size is not allowed here. %s argument: A format directive with a variable precision is not allowed here. %s argument: A format directive with a variable width is not allowed here. %s argument: The character '%c' is not a valid conversion specifier. %s argument: The character that terminates the format directive is not a valid conversion specifier. %s argument: The format string consumes more than one argument: %u argument. %s argument: The format string consumes more than one argument: %u arguments. %s argument: The string ends in the middle of a directive. %s: I/O error %s:%u:%u %s:%u:%u: cannot convert %s:%u:%u: incomplete character or shift sequence (stdin) Converts text from one encoding to another encoding.
 I/O error Informative output:
 Options controlling conversion problems:
 Options controlling error output:
 Options controlling the input and output format:
 Try '%s --help' for more information.
 Usage: %s [OPTION...] [-f ENCODING] [-t ENCODING] [INPUTFILE...]
 Usage: iconv [-c] [-s] [-f fromcode] [-t tocode] [file ...] Written by %s.
 cannot convert byte substitution to Unicode: %s cannot convert byte substitution to target encoding: %s cannot convert byte substitution to wide string: %s cannot convert unicode substitution to target encoding: %s cannot convert widechar substitution to target encoding: %s conversion from %s to %s unsupported conversion from %s unsupported conversion to %s unsupported or:    %s -l
 or:    iconv -l try '%s -l' to get the list of supported encodings Project-Id-Version: libiconv 1.15-pre1
Report-Msgid-Bugs-To: bug-gnu-libiconv@gnu.org
PO-Revision-Date: 2016-12-12 13:23+0200
Last-Translator: Toomas Soome <tsoome@me.com>
Language-Team: Estonian <linux-ee@lists.eenet.ee>
Language: et
MIME-Version: 1.0
Content-Type: text/plain; charset=UTF-8
Content-Transfer-Encoding: 8bit
X-Bugs: Report translation errors to the Language-Team address.
Plural-Forms: nplurals=2; plural=(n != 1);
   --byte-subst=VORMING        mitteteisendatavate baitide asendus
   --help                      väljasta see abiinfo ja lõpeta töö
   --unicode-subst=VORMING
                              mitteteisendavate Unikoodi sümbolite asendus
   --version                   väljasta versiooni info ja lõpeta töö
   --widechar-subst=VORMING
                              mitteteisendatavate mitmebaidi sümbolite asendus
   -c                          eemalda mitteteisendatavad sümbolid
   -f KODEERING, --from-code=KODEERING
                              sisendi kooditabel
   -l, --list                  väljasta toetatud kooditabelite nimekiri
   -s, --silent                keela probleemidest teatamine
   -t KODEERING, --to-code=KODEERING
                              väljundi kooditabel
 %s %s argument: Suuruse määranguga vorming ei ole siin lubatud. %s argument: Muutuva täpsusega vorming ei ole siin lubatud. %s argument: Muutuva pikkusega vorming ei ole siin lubatud. %s argument: Sümbol '%c' ei ole lubatud teisenduses. %s argument: Vormingi direktiivi lõpetav sümbol ei ole teisenduses lubatud. %s argument: Vormingu sõne nõuab enam kui ühte argumenti: %u argument. %s argument: Vormingu sõne nõuab enam kui ühte argumenti: %u argumenti. %s argument: Sõne lõppeb keset direktiivi. %s: S/V viga %s:%u:%u %s:%u:%u: ei saa teisendada %s:%u:%u: mittetäielik sümbol või nihkejärjend (standardsisend) Teisendab teksti ühest kooditabelist teise.
 S/V viga Infoväljund:
 Teisendamisel tekkivate probleemide kontrolli võtmed:
 Vigade väljundi kontrollimise võtmed:
 Sisendi ja väljundi vormingut kontrollivad võtmed:
 Lisainfo saamiseks kasutage '%s --help'.
 Kasutamine: %s [VÕTI...] [-f KODEERING] [-t KODEERING] [SISENDFAIL...]
 Kasutamine: iconv [-c] [-s] [-f koodist] [-t koodi] [fail ...] Kirjutanud %s.
 baidiasendust ei saa Unikoodi teisendada: %s baitide asendust ei saa sihttabelisse teisendada: %s baidiasendust ei saa mitmebaidiliste sümbolitega sõneks teisendada: %s unikoodi asendust ei saa sihttabelisse teisendada: %s widechar sümboleid ei saa sihttabelisse teisendada: %s teisendust tabelist %s tabelisse %s ei toetata teisendust tabelist %s ei toetata teisendust tabelisse %s ei toetata või:    %s -l
 või:    iconv -l Toetatud kooditabelite niekirja saate käsuga '%s -l' 