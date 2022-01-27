// teste Cesar aguardando ajuda
/*
     Programa em C para funcionamento do CI HX711 com motor de passo
      Conversor AD para célula de carga
      Cesar Costa
      Julho 2020

      #06/07/20 - o motor de passo está muito lento, tenho que procurar soluções online para poder ler o HX711 e passar os passos para o motor simultaneamente
       O problema esta da linha 350 ate a linha 375, onde ele lê o HX711 e movimenta o motor e atualiza o display, tudo isso tem que ser executado em 7,5 segundos

       % Solução para motor de passo % ?criar nova Classe para objeto  Sugiro "C_servo();",? o mesmo pode evoluir de passos para servo?
         Incluir as variaveis com limite de passos; limite de velocidade; direção; fins de cursos

*/
//==============================================================================================================================================================================================================================
// --- Mapeamento de Hardware ---
// PIN 0 AND 1 SERIAL DATA  //desativado serial
#include <LiquidCrystal.h>  // lcd(12, 11, 5, 4, 3, 2); //pinos do LCD 12=4-RS , 11=6-E , 5=11-D4 , 4= 12-D5 , 3=13-D6 , 2=14-D7
#include <EEPROM.h>         // escrever as taras para recuperar dados
#define  M1_LADO 9       // Define pino 9 como lado do drive motor 
#define  M1_PASSO 10     // Define pino 10 como passo para drive motor
#define  M1_VOLTA 3200        // mumero de passos por revolução
#define  ADDO  7            //Data Out CI HX711
#define  ADSK  6            //SCK   CI HX711
#define  LED  13            //Led indicador power on
#define  ATUADOR 8           //acionador do rele do atuador   "trocado julho de pino 9 para pino 8"
#define  TRAVA  A0          //laranja- botão swith pull down
#define  MENU  A1           //amarelo- botão swith pull down
#define  ESQUERDA A2        //verde- botão swith pull down
#define  DIREITA  A3        //Azul- botão swith pull down
#define  SAIR  A4           //Roxo- botão swith pull down
#define  FIM_CURSO  A5       // fim de curso do M1

//==============================================================================================================================================================================================================================
// --- Protótipo das Funções Auxiliares ---

void Motor1();                // movimentar motor de acordo com a direção, velocidade e diferença de M1possN M1possA -
void Medir();               //função para ler e imprimir os valores encontrados e acender led se 0
void LerMedia();            // calcula a x millis medida e posição
void LerAtual();            //calcula valor atual de angulo e peso
void ExibirLCD();           //exibi no LCD para as Funções LerMedia e LerAtual
void EEPROMWriteLong(int address, long value); //4 Bytes, grava o valor long na memoria
unsigned long EEPROMReadLong(int address); //4 Bytes, faz leitura do endereço de memoria e retorna o valor em unsigned long
unsigned long ReadCount();  //conversão AD do HX711
void Zerar1();              // zera no ponto inicial em cima
void Zerar2();              // identifica o valor no ponto dois para compesar com a Função F(X)=sin( X "angulo inclinação")* zerar2 "variavel encontrada"
void Trava();               // Trava ou destrava o atuador segura o papel
void MoveLivre();           //função para movimentar motor1
void GramaNewton();         // função para gravar na memoria fixa
int Teclas();              // ler as teclas no loop
void keyboard_menu();
void menu1();
void menu2();
void menu3();
void menu4();

//==============================================================================================================================================================================================================================
// --- Variáveis Globais ---
#define   menu_max   4
unsigned long convert=0;                     //  variavel recebida do HX711
unsigned long previousMillis = 0;            //  will store last time LED was updated HX711
unsigned long M1_previousMillis = 0;         // ultimo passo do motor 1
const long interval = 20;                    //  original 100 constants won't change:
long varB = 8364550;                         //  variavel B da equação "F(x)=a*x+b" = Tara zero
int varA = 901;                              //  variavel A da equação aplicando "varB-convert/varA" 0,001111 unidade de grama901= n
long grama = 0;                              //  VARIAVEL PARA TER PESO DA BALANÇA ATUALIADO
int gramaMax = 0;                            // peso maximo registrado
float gramaMaxNewton = 0;
int maxima = 0;                              // angulo do peso maximo
int varGrama = 0;                            //  variavel compensando inclinação da balança
float varNewton = 0;  
int media = 0;                               //
int contagem = 0;                            //
int unidadeM =1;                             // 1= GRAMA  0 = NEWTON
unsigned long currentMillis = 0;             //
bool zerar1 = LOW;                           //  registrador de execução 1 posição
long zerar2 = 1;                             //  registrador de execução 2 posição
int ponto1 = 0;                              //  definição do 1º ponto  , inicio da medida
int ponto2 = 90;                             //  definição do 2º ponto  , final da medida
int val;
int valD = 0;
int M1possN = 0;                             //  varievel de nova posição para motor1
int M1possA = 0;                             //  variavel para posição atual do motor1
int M1_RPM =2;                               // VELOCIDADE DO MOTOR 1 DE ROTAÇÃO POR MINUTO
int M1_tempo = (M1_VOLTA*M1_RPM)/60000;      // VELOCIDADE DO MOTOR 1 NUMERO DE PASSOS p/ REVOLUÇÃO *RPM / 60000 milisegundos
bool dr;                                     //  registrador do botão direito
bool es;                                     //  registrador do bottão esquerdo
// menus
int QTDEdeMenus = 3;                         //  numero de opções no menu principal
int menuPricipal = 1;                        //  posição atual do menu principal
bool sair;
int teclas = 0;
int menu_num = 1;
int sub_menu = 1;

// ---Variáveis alocação de Memoria ---
int  M_varB =10 ;                            //define o ponto de memmoria para salvar variavel long
int  M_zerar1 =20;                           //define o ponto de memmoria para salvar variavel bit 
int  M_zerar2 =30 ;                          //define o ponto de memmoria para salvar variavel long
int M_unidadeM=40;

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);       //pinos do LCD 12=4-RS , 11=6-E , 5=11-D4 , 4= 12-D5 , 3=13-D6 , 2=14-D7
bool atuador;                                // CONTROLE DO ATUADOR

//==============================================================================================================================================================================================================================
// --- Configurações Iniciais ---
void setup()
{
  lcd.begin(20, 4);                          //  iniciar modulo de LCD 
  pinMode(ADDO, INPUT);                      //  entrada para receber os dados
  pinMode(ADSK, OUTPUT);                     //  saída para SCK
  pinMode(M1_LADO, OUTPUT);                  //  saida do led
  pinMode(M1_PASSO, OUTPUT);                 //  saida do led
  digitalWrite(M1_PASSO, LOW);               // completa o passo do motor 
  pinMode(LED, OUTPUT);                      //  saida do led
  pinMode(TRAVA, INPUT_PULLUP);              //  botão pullup
  pinMode(MENU, INPUT_PULLUP);               //  botão pullup
  pinMode(ESQUERDA, INPUT_PULLUP);           //  botão pullup
  pinMode(DIREITA, INPUT_PULLUP);            //  botão pullup
  pinMode(SAIR, INPUT_PULLUP);               //  botão pullup
  pinMode(FIM_CURSO ,INPUT_PULLUP);          //  Swich do fim de curso Motor1
  pinMode(ATUADOR, OUTPUT);                  //  Saida para atuador
  valD = 0;                                  //  Posição atual do motor1 
  varB = EEPROMReadLong(M_varB);             //  carrega variavel com informações existente na memoria 
  zerar1 = EEPROM.read(M_zerar1);            //  carrega variavel com informações existente na memoria 
  zerar2= EEPROMReadLong(M_zerar2);          //  carrega variavel com informações existente na memoria 
  
  // Incluir Função para acertar posição do motor de passo pelo fim de curso 
  //"fazer movimentação para localizar a posição do motor e certificar que o mesmo esta em ponto 

} //end setup

//==============================================================================================================================================================================================================================
// --- Loop Infinito ---
void loop()
{
  keyboard_menu();                           //fazer leitura do teclado
  switch (menu_num)                          // ir para menu escolhido pelo teclado
  {
    case 1: menu1(); break;                  // Loop entra na Função do menu escolhido
    case 2: menu2(); break;
    case 3: menu3(); break;
    case 4: menu4(); break;
  } //end switch
      
  
} //end loop

//==============================================================================================================================================================================================================================

// --- Funções ---
//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
// --- Movimentar o motor de passo 

 // o milles faz os passo acontecer em um x periodo,    #####o problema que demora muito para ler o HX711 passando o tempo do passo####

void Motor1()  {
if (millis() - M1_previousMillis >= M1_tempo && M1possN != M1possA){

      if (M1possN > M1possA){
            digitalWrite(M1_LADO,LOW);
            M1possA = M1possA +1;}
      else {
            digitalWrite (M1_LADO, HIGH);
               M1possA = M1possA -1;}
      M1_previousMillis=millis();
      digitalWrite(M1_PASSO, HIGH);
      }
digitalWrite(M1_PASSO, LOW); 
}// end Motor1
      
//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
// --- Teclas --- faz a leitura do teclado, uma tecla por vez!
int Teclas() {
  if (!digitalRead(MENU))return 1;
  if (!digitalRead(ESQUERDA))return 2;
  if (!digitalRead(DIREITA))return 3;
  if (!digitalRead(SAIR))return 4;
  if (!digitalRead(TRAVA))return 5;
  else   return 0;
}//end Teclas

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
// -- Função do modulo HX711 - celula de carga

// função igual ao encontrao no data sheet do  HX711
unsigned long ReadCount()
{
  unsigned long Count = 0;
  unsigned char i;
  digitalWrite(ADSK, LOW);
  while(digitalRead(ADDO)) ;
  
  for (i = 0; i < 24; i++)
  {
    digitalWrite(ADSK, HIGH);
    Count = Count << 1;
    digitalWrite(ADSK, LOW);
    if (digitalRead(ADDO)) Count++;
  } //end for
      
  digitalWrite(ADSK, HIGH);
  Count = Count ^ 0x800000;
  digitalWrite(ADSK, LOW);
  return (Count);

} //end ReadCount

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
void LerMedia() {  // imprimir constantemente o peso da balança nas linhas 2 e 3
  

  if (millis() - previousMillis >= interval) { 
    previousMillis = millis();
    contagem = contagem + 1; //passar contando o quantas vezes soma o peso
    convert = convert + ReadCount(); // soma o pesso para tirar a media
   
    //    if (contagem == 10) { //exibe o resultado a cada 10 mediçoes
    //      contagem = 0 ; // zera a contagem
    //      convert = convert / 10;

    if (contagem == 2) { //exibe o resultado a cada 2 mediçoes
      contagem = 0 ; // zera a contagem
      convert = convert / 2;

      // processo para determinar sé o valor é possitivo ou negativo

      if (convert < varB) {
        convert = varB - convert; //
        grama = convert / varA;
        grama = grama * -1;
      }
      if (convert > varB) {
        convert = convert - varB; //
        grama = convert / varA;
      }
      if (convert == varB) {
        grama = 0;
      }
      convert = 0 ; //zera o convert
    }
  }
  if ( grama < 1 && grama > -1) { //acender led ao lado do display
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }//end if
 
     if (varGrama > gramaMax){
      gramaMax = varGrama;
      maxima = val;
     }
  ExibirLCD();
}//end LerMedia

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

void LerAtual() {  // atualizar o peso da balança nas linhas 2 e 3

  // verificar se o valor é positivo ou negativo
  convert =  ReadCount(); // soma o pesso para tirar a media de 2
  if (convert < varB) {
    convert = varB - convert; //
    grama = convert / varA;
    grama = grama * -1;
  }
  if (convert > varB) {
    convert = convert - varB; //
    grama = convert / varA;
  }
  if (convert == varB) {
    grama = 0;
  }
  if ( grama < 1 && grama > -1) { //acender led ao lado do display se o valor for 0
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }//end if
   varGrama = grama   // valor em grama
     if (varGrama > gramaMax){
      gramaMax = varGrama;
      maxima = val;
     }
  ExibirLCD();
}//end LerAtual

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

void ExibirLCD() // inicio da função  exibe para o LerMedia e LerAtual
{
  
    lcd.setCursor(0, 2);
  lcd.write("D= ");
  lcd.write("     ");
  lcd.setCursor(3, 2);
  lcd.print(val);
  lcd.setCursor(6, 2);
  lcd.write("/peso=        ");
  lcd.setCursor(13, 2);
  lcd.print(varNewton,3);
  lcd.setCursor(0, 3);
  lcd.write("MAX=                ");
  lcd.setCursor(7, 3);
  lcd.print(gramaMaxNewton,3);      
  }else{  //exibir em grama
    lcd.setCursor(0, 2);
  lcd.write("D= ");
  lcd.write("     ");
  lcd.setCursor(3, 2);
  lcd.print(val);
  lcd.setCursor(6, 2);
  lcd.write("/peso=        ");
  lcd.setCursor(13, 2);
  lcd.print(varGrama);
  lcd.setCursor(0, 3);
  lcd.write("MAX=                ");
  lcd.setCursor(7, 3);
  lcd.print(gramaMax);      
  }
  
}  //end ExibirLCD

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

void Medir() {  //inicio da função de leitura
  /*   *criar variaveis para:
    1º Gerar array para apresentar Grafico tempo/angulo X peso

  */
  //variaveis locais
  int grafico[90];
  int leitura[2];
  int qtdeLeitura = 0;
  int media = 0;
 unsigned long desceMile = 0;
  int fim = 0;
  int tempo;
  
  // checar posição de motor paralela a folha e balança zerada
  if (!zerar1 || val != 0) {
    lcd.clear();
    lcd.print("  Erro == 02");// faltou zerar a balança ou iniciar da posição 0
    lcd.setCursor(0, 2);
    lcd.print(" ZERAR PONTOS 1 e 2 ");
    delay(1200);
    menu_num = 2;
    return;
  }
  //checar atuador ativo
  else if (!atuador) {
    lcd.clear();
    lcd.print("  Erro == 03");// faltou travar amostra
    lcd.setCursor(0, 2);
    lcd.print("   TRAVAR AMOSTRA   ");
    delay(1200);    
    return;
  } else {
gramaMax = 0;
    //LeD indicador de display
    delay(1000);
    //registar tempo de inicio
    tempo = millis();
    
    // tem que se movimentar 90° no periodo total de  7500 miles segundos.
    // aproximadamente 83 miles por grau.
       M1_tempo=9;   // para avançar um passo a cada 9 miles segundos.
    //iniciar descida da celula de carga
    for (int i=0; i <= 90 ; i++) //descer por passo e registar a cada medida no array
    {
          val=i;
          
          LerAtual();
          M1possN = map(i, 0, 90, 0, 800); // ajustar o valor para inclinação do Motor compensando diferença
         Motor1();
          
      grafico[i] = varGrama;        //incli a informação na tabela a cada grau
    }
    
    fim = varGrama;
    //ao finalizar curso comparar com tempo subtraindo resultado para imprimir na tela
    tempo = millis() - tempo;
    tempo= tempo/1000;
    //exibir media  qtde de posições acima de 1 grama
    for (int i = 0 ; i <= 90 ; i++) {
      if (grafico[i] > 0) {
        media = media + grafico[i];
        qtdeLeitura++;
      }
    }
    media = media / qtdeLeitura;
  
    //retornar motor para inicio
        M1_tempo=1;
    for(int i =90;i>=0; i--)
      { 
           
          val = i;
          M1possN = map(i, 90, 0, 800, 0); // scale it to use it with the Motor (value between 0 and 180)
           Motor1();
          delayMicroseconds(50);
       }
   
      Trava();
      delay (250);
        //finalizar  exibindo balança #tempo , media, Max, final
      
   
      if(unidadeM==0){
         varNewton=fim*0.009807;
    gramaMaxNewton=gramaMax*0.009807;
           lcd.clear();
      lcd.print("**   RESULTADOS   **");
      lcd.setCursor(0,1);
      lcd.print("HOLD MAX = ");      
      lcd.print(gramaMaxNewton,3);       
      lcd.setCursor(0,2);    
      lcd.print("ANG. MAX = ");
      lcd.print(maxima);
      lcd.setCursor(0,3);
      lcd.print("HOLD 90  = ");
      lcd.print(varNewton,3);
      }else{
      lcd.clear();
      lcd.print("**   RESULTADOS   **");
      lcd.setCursor(0,1);
      lcd.print("HOLD MAX = ");      
      lcd.print(gramaMax);       
      lcd.setCursor(0,2);    
      lcd.print("ANG. MAX = ");
      lcd.print(maxima);
      lcd.setCursor(0,3);
      lcd.print("HOLD 90  = ");
      lcd.print(fim);
      }
      bool aguardar=HIGH;
      while(aguardar){  
            if(!digitalRead(TRAVA)){
                  aguardar=LOW;
                  Trava();
            }else if(!digitalRead(SAIR))aguardar=LOW;
      }//end while
      lcd.clear();
      delay(200);

  }
 sub_menu=2;
  //imprimir array otimizando linha
}//end Medir



//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
//Zerar balança em cima
void Zerar1() {
  LerMedia();

  varB = ReadCount(); //gravar variavel TARA a balança
  contagem = 0;
  convert = 0;
  zerar1 = HIGH;
 
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.println("      ZERANDO        ");
  lcd.println("   ponto  um    ");
  delay(500);
  LerMedia();
  previousMillis = currentMillis;

}// end Zerar1

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
//Zerar balança em baixo
void Zerar2() {

  if (zerar1 == HIGH) {
    LerMedia();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.println("      ZERANDO        ");
    lcd.println("  ponto dois  ");
    delay(500);
  }
  else {
    LerMedia();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("   TEM QUE ZERAR       "); lcd.setCursor(0, 1);
    lcd.print("  PONTO  UM   ");
    lcd.setCursor(3, 3);
    lcd.print(" ERRO - 01");
    delay(1000);
    LerMedia();
  }

}// end Zerar2

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
void Trava() {

  while (!digitalRead(TRAVA)) delay(250);
  atuador = !atuador;
  if (atuador == HIGH) {
    digitalWrite(ATUADOR, HIGH);
    delay(100);
    lcd.begin(20, 4);
  } else {
    digitalWrite(ATUADOR, LOW);
    delay(100);
    lcd.begin(20, 4);
  }
}//end  TRAVA

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

void MoveLivre() {
  sair = HIGH; 
  lcd.setCursor(0, 0);
  lcd.print("   Move Livre      ");
  do {
      //
      if (!digitalRead(DIREITA) && val < 90) {
      lcd.setCursor(0, 1);
      lcd.print("> Direita >      ");
      delay(20);
      val = val + 1;
    }
    else if (!digitalRead(ESQUERDA) && val > 0) {
      lcd.setCursor(0, 1);
      lcd.print("< Esquerda <     ");
      delay(20);
      val = val - 1;
    } else {
      lcd.setCursor(0, 1);
      lcd.print("                    ");
          M1_tempo=0;
    }
    M1possN = map(val, 0, 90, 0, 800); // scale it to use it with the Motor (value between 0 and 180)              
    Motor1();                  // sets the Motor position according to the scaled value
    LerMedia();
    if (!digitalRead(SAIR)) {
      sair = LOW;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("  SAINDO MOVE LIVRE   ");
      delay(700);
    }
  } while (sair == HIGH);
  sub_menu = 1;
  return;

}// end MoveLivre

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
void GramaNewton(){
  //criar codigo
  return;
}// end GramaNewton

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

void EEPROMWriteLong(int address, long value) {
   byte four = (value & 0xFF);
   byte three = ((value >> 8) & 0xFF);
   byte two = ((value >> 16) & 0xFF);
   byte one = ((value >> 24) & 0xFF);
   EEPROM.write(address, four);
   EEPROM.write(address + 1, three);
   EEPROM.write(address + 2, two);
   EEPROM.write(address + 3, one);
}//end EEPROMWriteLong

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO


unsigned long EEPROMReadLong(int address) {
   long four = EEPROM.read(address);
   long three = EEPROM.read(address + 1);
   long two = EEPROM.read(address + 2);
   long one = EEPROM.read(address + 3);
   return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}//end EEPROMReadLong

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

void keyboard_menu()
{
  if (!digitalRead(ESQUERDA) && sub_menu == 1)
  {
    while(!digitalRead(ESQUERDA)){}
    delay(150);
    lcd.clear();
     if (menu_num > 1) menu_num -= 1;
  } //end ESQUERDA
  if (!digitalRead(DIREITA) && sub_menu == 1)
  {
    while(!digitalRead(DIREITA)){}
    delay(150);
    lcd.clear();
    if (menu_num < menu_max) menu_num += 1;
  } //end DIREITA
  if (!digitalRead(MENU))
  {
    while(!digitalRead(MENU)){}
    delay(150);
    lcd.clear();
    if (sub_menu <= 2) sub_menu = 2;
  } //end MENU
  if (!digitalRead(SAIR))
  {
    while(!digitalRead(SAIR)){}
    delay(150);
    lcd.clear();
    if (sub_menu > 0) sub_menu = 1;
  } //end SAIR

} //end keyboard_menu

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

void menu1()
{
  switch (sub_menu)
  {
    case 1:
      lcd.setCursor(0, 0);
      lcd.print("     AUTOMATICO    >");
      
      break;
    case 2:
      lcd.setCursor(0,0);
      lcd.print("     AUTOMATICO    ");
      lcd.setCursor(0,1);
      lcd.print("                   ");
      //criar codigo
      break;
  }


} //end menu1


//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

void menu2()
{
   //criar MENU Zerando
      break;
}
} //end menu2

//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO

void menu3()
{
  switch (sub_menu)
  {
  //criar codigo
      break;
  }


} //end menu3


//OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO


void menu4()
{
  switch (sub_menu)
  {
    case 1:
      lcd.setCursor(0, 0);
      lcd.print("<  MOVIMENTO LIVRE  ");
      lcd.setCursor(0, 1);
      lcd.print("                ");
      break;
    case 2:
      MoveLivre();
      break;
  }


} //end menu4
