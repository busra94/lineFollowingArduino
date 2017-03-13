/*
 * QTRRC sensörün kendi  kütüphanesini entegre ettik.
*/
#include <QTRSensors.h>
/* 
 *  PID algoritmasında kullanılan Kp Kd ve Ki değerlerini tanımlıyoruz.
 *  Bu değerleri deneme yanılma yoluyla hesapladık.
 *  İlk olarak Kp değerini test ettik ve robotumuzun kafa sallama durumunu en aza indirgeyecek değeri bulduk.
 *  Daha sonra Kd değerini test ettik robotun dönüşerdeki hassasiyetini arttırdık. Böylece döüşlerde yoldan çıkma ihtimalini yok ettik.
 *  Son olarak da Ki değerini ekleyip duyarlılığını keskin dönüşlerde de arttırmış olduk.
*/

#define Kp 0.039 //0.099
#define Kd 0.23 // 0.315
#define Ki 0.0015 // 0

/*
 * Yine algoritma gereği ilk hızını ve çıkabileceği maksimum hızını belirledik.
 * Bu değerleri de yine deneme yanılma yoluyla bulduk.
 * Bu değerler labaratuvar şartlarımıza uygun en ideal değerlerdir.
 * Dilerseniz bunları orantılı bir şekilde arttırabilirsiniz.
*/
#define maxRightSpeed 100 
#define maxLeftSpeed 100 
#define defaultRightSpeed 30 
#define defaultLeftSpeed 30  

/*
 * Kaç tane sensör kullanılacaksa onları tanımladık. Dilerseniz bunu sekize çıkarabilirsiniz.
 * QTRRC sensör kullanıma bağlı olarak sensör çıkışlarının düşük kalması için 2500 us bekleme veriyoruz.
 * QTRRC sensör kullanımına bağlı olarak yayıcı pinin çıkışını 2 ye veriyoruz.
*/
#define numberOfSensors  6     
#define TIMEOUT       2500  
#define EMITTER_PIN   2  

/*
 * Aşağıda sağ ve sol motorların motor sürücüden arduinoya olan bağlantılarını verdik.
 * Bu bağlantılar arduino nano içindir.
*/
#define rightMotorForward 7
#define rightMotorBack 8
#define PWM_RIGHT 3
#define leftMotorForward 4
#define leftMotorBack 5
#define PWM_LEFT 9
#define STBY 6

/*
 * Aşağıda bir qtrrc nesnesi tanımalayıp sensör dizimizi, sayısını , bekleme süremizi ve yayıcıyı içine dahil ettik.
*/
QTRSensorsRC qtrrc((unsigned char[]) {  A0, A1, A2, A3, A4, A5} ,numberOfSensors, TIMEOUT, EMITTER_PIN); 

unsigned int sensorValues[numberOfSensors];

/*
 * setup bloğunda motor pinlerini çıkışa atıyoruz.
*/
void setup()
{
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBack, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBack, OUTPUT);
  pinMode(PWM_LEFT, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY,HIGH);
  Serial.begin(9600);
  
  /*
   * robotun çizgiye adapte olabilmesi için kalibrasyon metodunu kullandık.
  */
  int i;
  
  for (int i = 0; i < 100; i++) 

  
   qtrrc.calibrate();   
   delay(20);
   waiting();
   delay(2000); 
    
    
  } 
/*
 * PİD algoritması için gerekli hata integral değerlerini tanımladık.
 * Siyah çizgide ise isBlack beyaz çizgide ise isWhite değeri kullanılmak üzere tanımladık. Nedeni ise readline metodunda bu zemin bilgisi girilir. 
*/
int previousError = 0;
int previousIntegral = 0;
bool isWhite=0;
bool isBlack=1;

void loop()
{
  
  unsigned int sensors[6];
  
  int position =qtrrc.readLine(sensors,QTR_EMITTERS_ON,isWhite); 
  
  
  int error = 2500- position;
  
  
  int Integral = error+previousIntegral;

  /*
   * Motorların hızını sensörlerden okuduğumuz değerlere göre ayarladık.
   * Sonrasında motorların hızlarının negatif değere düşmemesi ve maksimum hızı aşmaması için kontroller yaptık.
  */
  int speedOfMotor = Kp * error + Kd * (error - previousError) + Ki * Integral;
  previousError = error;
  previousIntegral=Integral;

  int speedOfRight = defaultRightSpeed + speedOfMotor;
  int speedOfLeft = defaultLeftSpeed - speedOfMotor;
  
  if (speedOfRight > maxRightSpeed ) speedOfRight = maxRightSpeed; 
  if (speedOfLeft > maxLeftSpeed ) speedOfLeft = maxLeftSpeed; 
  if (speedOfRight < 0) speedOfRight = 0; 
  if (speedOfLeft < 0) speedOfLeft = 0; 
  
   {
   /*
    * Motorların yürümesi için ileri yöndeki hareketini HIGH, geri yöndeki hareketini LOW yaptık.
   */
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBack, LOW);
  analogWrite(PWM_RIGHT, speedOfRight);
  
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBack, LOW);
  analogWrite(PWM_LEFT, speedOfLeft);
}

}
void waiting(){
  digitalWrite(STBY,HIGH);
}
