��    &      L  5   |      P  8   Q  B   �  A   �  6     H   F  I   �  F   �  9      7   Z  6   �  M   �  L     O   d  H   �  {   �     y  �   �  e   `  :   �    	  �  
  �  �     �     �  <   �  1   �  &   "     I  "   X  9   {  I   �  �   �     �     �     �     �     �  �  �  _   �  f   1  d   �  Q   �  �   O  �   �  �   k  a   �  _   U  e   �  m     ~   �  f     �   o  �   �     �  �  �  �   �  U   e    �  \  �  �  !  '   �#     �#  �   �#  E   �$  W   �$     A%  ?   P%  h   �%  �   �%  �   �&  "   ~'  )   �'  #   �'     �'  ,   (              !          #                                                      %                                        
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
PO-Revision-Date: 2020-04-16 11:32+0300
Last-Translator: Yuri Chornoivan <yurchor@ukr.net>
Language-Team: Ukrainian <trans-uk@lists.fedoraproject.org>
Language: uk
MIME-Version: 1.0
Content-Type: text/plain; charset=UTF-8
Content-Transfer-Encoding: 8bit
X-Bugs: Report translation errors to the Language-Team address.
X-Generator: Lokalize 20.07.70
Plural-Forms: nplurals=1; plural=0;
   -E                        (буде проігноровано, для сумісності)
   -V, --version               вивести інформацію про версію та вийти
   -V, --version             вивести інформацію про версію та вийти
   -c, --context=КОНТЕКСТ    вказати контекст для MSGID
   -d, --domain=TEXTDOMAIN   отримати перекладене повідомлення з текстового домену TEXTDOMAIN
   -d, --domain=TEXTDOMAIN   отримати перекладені повідомлення з текстового домену TEXTDOMAIN
   -e                        увімкнути розгортання деяких послідовностей екранування
   -h, --help                  вивести довідку та завершити роботу
   -h, --help                вивести довідку та завершити роботу
   -n                        придушити кінцевий символ нового рядка
   -v, --variables             виводити змінні, що зустрічаються у SHELL-FORMAT
   КІЛЬКІСТЬ                 вибрати однину/множину на основі цього значення
   MSGID MSGID-PLURAL        перекласти MSGID (однина) / MSGID-PLURAL (множина)
   [TEXTDOMAIN]              отримати перекладене повідомлення з текстового домену TEXTDOMAIN
   [TEXTDOMAIN] MSGID        отримати перекладене повідомлення відповідно до
                            MSGID з текстового домену TEXTDOMAIN
 Bruno Haible Авторські права належать Free Software Foundation, Inc., %s
Умови ліцензування викладено у GPLv3+: GNU GPL версії 3 або новішій, <%s>
Це вільне програмне забезпечення: ви можете вільно змінювати і поширювати його.
Вам не надається ЖОДНИХ ГАРАНТІЙ, окрім гарантій передбачених законодавством.
 Відображає переклад текстового повідомлення, граматична форма якого залежить
від числа.
 Відображає переклад текстового повідомлення.
 Якщо параметр ДОМЕН_ТЕКСТУ не вказаний, використовується домен, визначений у
змінній середовища TEXTDOMAIN. Якщо файл з перекладеними повідомленнями
відсутній у стандартному каталозі, можна вказати інший каталог за допомогою
змінної середовища TEXTDOMAINDIR.
Стандартний каталог пошуку: %s
 Якщо не вказаний параметр ДОМЕН_ТЕКСТУ, використовується домен, встановлений
у змінній середовища TEXTDOMAIN. Якщо файл з перекладеними повідомленнями
відсутній у типовому каталозі, можна вказати інший каталог за допомогою
змінної середовища TEXTDOMAINDIR.
При використанні з ключем -s, поведінка програми схожа на поведінку програми
«echo». Але замість простого копіювання аргументів у стандартний вивід,
виводяться їх переклади з вказаного домену.
Стандартний каталог пошуку: %s
 У звичайному режимі роботи, стандартний ввід копіюється на стандартний вивід,
де посилання на змінні середовища у формі $VARIABLE або ${VARIABLE}, 
замінюються відповідними значеннями. Якщо вказано SHELL-FORMAT,
будуть замінюватись лише змінні, що вказані у SHELL-FORMAT; у іншому
випадку будуть замінюватись усі змінні середовища, що зустрічаються у
стандартному вводі.
 Інформативний вивід:
 Режим роботи:
 Про вади слід повідомляти за допомогою системи стеження за вадами: <%s>
або електронною поштою на адресу <%s>.
 Замінює значення змінних середовища.
 Віддайте команду «%s --help», щоб дізнатися більше.
 Ulrich Drepper Використання: %s [ПАРАМЕТР] [SHELL-FORMAT]
 Використання: %s [ПАРАМЕТР] [ДОМЕН_ТЕКСТУ] MSGID MSGID-PLURAL ЧИСЛО
 Використання: %s [ПАРАМЕТР] [[ДОМЕН_ТЕКСТУ] MSGID]
або:          %s [ПАРАМЕТР] -s [MSGID]...
 При використанні --variables, стандартний ввід ігнорується, а вивід
складається зі змінних середовища, які вказані у SHELL-FORMAT, по одній на рядок.
 Автор програми - %s.
 помилка при читанні "%s" відсутні аргументи стандартний ввід надто багато аргументів 