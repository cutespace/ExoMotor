��    *      l  ;   �      �  �   �  �   �  �   >  c   �     A     Z  R  g  N   �  &   	  O   0     �  #   �  !   �  *   �  D   �  @   D	  %   �	  &   �	  &   �	  (   �	     "
     =
  6   I
     �
  (   �
  '   �
  4   �
  4     &   I  /   p  /   �  7   �  -     %   6  %   \  "   �     �  .   �  #   �  '        6  �  >  �   *  �      �   �  `   �          %  �  E  _   �  +   ,  V   X     �  "   �  0   �  B     N   X  I   �  0   �  +   "  )   N  .   x  "   �     �  M   �     (  )   >  6   h  6   �  6   �  +     0   9  0   j  >   �  3   �  *     1   9  +   k     �  4   �  '   �  +        @                	   )                                 #          
                            !         '                  (                                         $          "          *             &   %          --usestd3asciirules   Enable STD3 ASCII rules
      --no-alabelroundtrip  Disable A-label roundtrip for lookups
      --debug               Print debugging information
      --quiet               Silent operation
   -T, --tr46t               Enable TR46 transitional processing
  -N, --tr46nt              Enable TR46 non-transitional processing
      --no-tr46             Disable TR46 processing
   -d, --decode              Decode (punycode) domain name
  -l, --lookup              Lookup domain name (default)
  -r, --register            Register label
   -h, --help                Print help and exit
  -V, --version             Print version and exit
 A-label roundtrip failed Charset: %s
 Command line interface to the Libidn2 implementation of IDNA2008.

All strings are expected to be encoded in the locale charset.

To process a string that starts with `-', for example `-foo', use `--'
to signal the end of parameters, as in `idn2 --quiet -- -foo'.

Mandatory arguments to long options are mandatory for short options too.
 Internationalized Domain Name (IDNA2008) convert STRINGS, or standard input.

 Try `%s --help' for more information.
 Type each input string on a line by itself, terminated by a newline character.
 Unknown error Usage: %s [OPTION]... [STRINGS]...
 could not convert string to UTF-8 could not determine locale encoding format domain label has character forbidden in non-transitional mode (TR46) domain label has character forbidden in transitional mode (TR46) domain label has forbidden dot (TR46) domain label longer than 63 characters domain name longer than 255 characters input A-label and U-label does not match input A-label is not valid input error libiconv required for non-UTF-8 character encoding: %s out of memory punycode conversion resulted in overflow punycode encoded data will be too large string contains a context-j character with null rule string contains a context-o character with null rule string contains a disallowed character string contains a forbidden context-j character string contains a forbidden context-o character string contains a forbidden leading combining character string contains forbidden two hyphens pattern string contains invalid punycode data string contains unassigned code point string could not be NFC normalized string encoding error string has forbidden bi-directional properties string is not in Unicode NFC format string start/ends with forbidden hyphen success Project-Id-Version: libidn2 2.3.3
Report-Msgid-Bugs-To: bug-libidn2@gnu.org
PO-Revision-Date: 2024-09-23 13:06-0300
Last-Translator: Rafael Fontenelle <rafaelff@gnome.org>
Language-Team: Brazilian Portuguese <ldpbr-translation@lists.sourceforge.net>
Language: pt_BR
MIME-Version: 1.0
Content-Type: text/plain; charset=UTF-8
Content-Transfer-Encoding: 8bit
Plural-Forms: nplurals=2; plural=(n > 1)
X-Generator: Gtranslator 46.1
X-Bugs: Report translation errors to the Language-Team address.
       --usestd3asciirules   Habilita regras ASCII STD3
      --no-alabelroundtrip  Desabilita ida e volta de A-label para procuras
      --debug               Mostra informações de depuração
      --quiet               Operação silenciosa
   -T, --tr46t               Habilita processamento transicional de TR46
  -N, --tr46nt              Habilita processamento não transicional de TR46
      --no-tr46             Desabilita processamento de TR46
   -d, --decode              Decodifica (punycode) nome de domínio
  -l, --lookup              Procura nome de domínio (padrão)
  -r, --register            Registra o rótulo
   -h, --help                Mostra ajuda e sai
  -V, --version             Mostra versão e sai
 Ida e volta com A-label falhou Codificação de caractere: %s
 Interface de linha de comando para a implementação do Libidn2 de IDNA2008.

Espera-se que todas as strings sejam codificadas no codificação de
caractere do locale.

Para processar string que inicie com "-", por exemplo "-foo", use "--"
para sinal no fim de parâmetros, como em "idn2 --quiet -a -- -foo".

Argumentos obrigatórios para opções longas são também são para as curtas.
 Nome de domínio internacionalizado (IDNA2008) para conversão de STRINGS ou entrada padrão.

 Tente "%s --help" para mais informações.
 Digite cada string de entrada em uma linha, terminada por um caractere de linha nova.
 Erro desconhecido Uso: %s [OPÇÃO]... [STRINGS]...
 não foi possível converter a string para UTF-8 não foi possível determinar o formato de codificação do locale rótulo de domínio possui caractere proibido no modo não transicional (TR46) rótulo de domínio possui caractere proibido no modo transicional (TR46) rótulo de domínio possui ponto proibido (TR46) rótulo de domínio maior que 63 caracteres nome de domínio maior que 255 caracteres A-label e U-label de entrada não correspondem A-label de entrada não é válido erro na entrada libiconv necessário para codificação de caracteres diferentes de UTF-8: %s memória insuficiente a conversão punycode resultou em estouro os dados codificados em punycode serão grandes demais a string contém um caractere context-j com regra nula a string contém um caractere context-o com regra nula a string contém um caractere desautorizado a string contém um caractere context-j proibido a string contém um caractere context-o proibido a string contém um caractere de combinação inicial proibido a string contém padrões proibidos de dois hifenes a string contém dados punycode inválidos a string contém ponto de código não atribuído a string não pôde ser normalizada com NFC erro de codificação da string a string possui propriedades bidirecionais proibidas a string não é um formato NFC Unicode a string inicia/termina com hífen proibido sucesso 