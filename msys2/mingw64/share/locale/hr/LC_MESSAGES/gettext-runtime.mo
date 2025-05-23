��    &      L  5   |      P  8   Q  B   �  A   �  6     H   F  I   �  F   �  9      7   Z  6   �  M   �  L     O   d  H   �  {   �     y  �   �  e   `  :   �    	  �  
  �  �     �     �  <   �  1   �  &   "     I  "   X  9   {  I   �  �   �     �     �     �     �     �    �  >     J   Q  H   �  =   �  D   #  C   h  H   �  :   �  8   0  G   i  Q   �  C     M   G  D   �  �   �     ]  �   j  F   7  !   ~    �  �  �  �  �     8     M  E   b  -   �  9   �       %     ;   E  N   �  �   �     n     {     �     �     �              !          #                                                      %                                        
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
 error while reading "%s" missing arguments standard input too many arguments Project-Id-Version: gettext-runtime 0.24-pre1
Report-Msgid-Bugs-To: bug-gettext@gnu.org
PO-Revision-Date: 2025-03-26 20:56-0700
Last-Translator: Božidar Putanec <bozidarp@yahoo.com>
Language-Team: Croatian <lokalizacija@linux.hr>
Language: hr
MIME-Version: 1.0
Content-Type: text/plain; charset=UTF-8
Content-Transfer-Encoding: 8bit
Plural-Forms: nplurals=3; plural=(n%10==1 && n%100!=11 ? 0 : n%10>=2 && n%10<=4 && (n%100<10 || n%100>=20) ? 1 : 2);
X-Bugs: Report translation errors to the Language-Team address.
X-Generator: Vim9.1
   -E                        (zanemareno zbog kompatibilnosti)
   -V, --version               informira o inačici ovog programa i iziđe
   -V, --version             informira o inačici ovog programa i iziđe
   -c, --context=KONTEKST    specifikacija konteksta za MSGID
   -d, --domain=TEKSTDOMENA  preuzme prevedene poruke iz TEKSTDOMENE
   -d, --domain=TEKSTDOMENA  traži prevedene poruke iz TEKSTDOMENE
   -e                        omogući ekspanziju nekih kontrolnih nizova
   -h, --help                  pokaže ovu pomoć i iziđe
   -h, --help                pokaže ovu pomoć i iziđe
   -n                        ne ostavlja prazni redak na kraju datoteke
   -v, --variables             ispiše varijable koje se javljaju u FORMAT-LJUSKE
   BROJ                      odabere formu singular/plural po BROJU
   MSGID MSGID-PLURAL        prevede MSGID (singular) / MSGID-PLURAL (plural)
   [TEKSTDOMENA]             preuzme prevedene poruke iz TEKSTDOMENE
   [TEKSTDOMENA] MSGID       preuzme prevedenu poruku koja odgovara
                              navedenom MSGID-u iz TEKSTDOMENE
 Bruno Haible Copyright (C) %s Free Software Foundation, Inc.
Licenca GPLv3+: GNU GPL inačica 3 ili novija <%s>
Ovo je slobodan softver: slobodno ga mijenjajte i dijelite.
NEMA JAMSTVA do granica dopuštenih zakonom.
 Ispiše prijevod tekstne poruke čiji gramatički oblik ovisi o broju
 Ispiše prijevod tekstne poruke.
 Ako parametar TEKSTDOMENA nije naveden, domenu određuje varijabla okruženja
TEXTDOMAIN. Ako katalog poruka nije pronađen u uobičajenom direktoriju,
alternativna lokacija može se zadati varijablom okruženja TEXTDOMAINDIR.

Uobičajeni direktorij za pretragu je: %s
 Ako parametar TEKSTDOMENA nije naveden, domenu određuje varijabla okruženja
TEXTDOMAIN. Ako katalog poruka nije pronađen u uobičajenom direktoriju,
alternativna lokacija može se zadati varijablom okruženja TEXTDOMAINDIR.

Kad se koristi opcija -s, program se ponaša kao naredba ‘echo’;
ali umjesto jednostavnog ispisa svih svojih argumenata, program ispiše
njihove prijevode ako ih nađe u odabranom katalogu (TEKSTDOMENA).

Uobičajeni direktorij za pretragu je: %s
 U običnom načinu rada standardni ulaz kopira se na standardni izlaz,
s time da se referencije na varijable okruženja u formatu $VARIJABLA ili
${VARIJABLA} zamijene odgovarajućim vrijednostima. Ako je navedeni argument
FORMAT-LJUSKE, onda će samo one varijable okruženja s referencijom u
FORMAT-LJUSKE biti zamijenjene; u suprotnom, sve varijable okruženja koje se
pojavljuju na standardnom ulazu bit će zamijenjene.
 Informativni izlaz:
 Način rada (mode):
 Greške prijavite u ‘bug tracker’ na <%s>
ili e-poštom na <%s>.
 Zamjenjuje vrijednosti varijabli okruženja.
 Utipkajte ‘%s --help’ za pomoć i više informacija.
 Ulrich Drepper Uporaba: %s [OPCIJA] [FORMAT-LJUSKE]
 Uporaba: %s [OPCIJA] [TEKSTDOMENA] MSGID MSGID-PLURAL BROJ
 Uporaba: %s [OPCIJA] [[TEKSTDOMENA] MSGID]
    ili: %s [OPCIJA] -s [MSGID]...
 Ako se koristi --variables, standardni ulaz je zanemaren, a izlaz se
sastoji od onih varijabli okruženja s referencijom u FORMAT-LJUSKE,
po jedna po retku.
 Napisao %s.
 greška pri čitanju „%s’ nema argumenata standardni ulaz previše argumenata 