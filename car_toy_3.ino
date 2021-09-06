#include <Servo.h>

Servo steering;  // рулевое управление

/*
void servo_on()
{
digitalWrite(RELAY_BREAK, LOW);
}

void servo_off()
{
digitalWrite(RELAY_BREAK, HIGH);
}*/

#define COLLISION_FWD "Впереди препятствие"
#define COLLISION_SUN "Засветка датчика"
#define MOVE_FWD "Движение вперед"

int dir_frvd = 6;//управление н-мостом
int dir_back = 5;
int wheel = 3;//датчики холла на колесе
int wheel2 = 4;
int photodiode = 14;//датчик отраженного лазера
int laser_pwm = 11;//лазер с возможностью шим


int state = 0;// вспомогательная переменная для энкодера

int photo_state = 0;// уровень на фотодиоде бампера
int sleep_timer = 0;// таймер включения лазера в режиме ожидания 
unsigned long currentMillis = 0;// текущее время в мс
long cInt_delay = 0;

int move_cycle_enable = 0;// разрешение на сканирование препятствий до и во время движения.

int background = 980;//фоновый уровень
int background_prev_ok = background;
int collision = 0;// флаг наличия препятствия
int stat_robot = 0;
int test_position = 0; // флаг проверки позиции в ждущем режиме
int init_skip = 0;// счетчик подстройки фона в начале работы, без определения столкновений
int need_rotate = 0;// ожидаемое значение вращения колеса

void setup() 
{
    pinMode(dir_frvd, OUTPUT);
    pinMode(dir_back, OUTPUT);
    pinMode(laser_pwm, OUTPUT);
    pinMode(wheel, INPUT);
    pinMode(wheel2, INPUT);
    pinMode(photodiode, INPUT);
    
    digitalWrite(dir_frvd, LOW);
    digitalWrite(dir_back, LOW);
    
  // put your setup code here, to run once:
    steering.attach(9);
    steering.write(90);
    delay(2500);


    digitalWrite(laser_pwm, HIGH);
 
    Serial.begin(9600);

    background = analogRead(photodiode);//задаем фоновое значение

}

void robot_drive_forward()
{
   digitalWrite(dir_frvd, HIGH);
   digitalWrite(dir_back, LOW); 
}
void robot_drive_forward_left()
{
   steering.write(80);//80-(90)-100
   digitalWrite(dir_frvd, HIGH);
   digitalWrite(dir_back, LOW); 
}
void robot_drive_forward_right()
{
   steering.write(100);//80-(90)-100
   digitalWrite(dir_frvd, HIGH);
   digitalWrite(dir_back, LOW); 
}

void robot_drive_backward()
{
   digitalWrite(dir_frvd, LOW);
   digitalWrite(dir_back, HIGH);
}
void robot_drive_backward_left()
{
   steering.write(80);//80-(90)-100
   digitalWrite(dir_frvd, LOW);
   digitalWrite(dir_back, HIGH); 
}
void robot_drive_backward_right()
{
   steering.write(100);//80-(90)-100
   digitalWrite(dir_frvd, LOW);
   digitalWrite(dir_back, HIGH); 
}

void robot_stop()
{
   digitalWrite(dir_frvd, LOW);
   digitalWrite(dir_back, LOW);
}

void steering_default()
{
   steering.write(90);//80-(90)-100
}

int check_collision()//провека на столкновение впереди
{
   photo_state = analogRead(photodiode);//читаем датчик препятствия
   
   if((background >= photo_state+16 || background  < photo_state - 16 ))//значение датчика отлично от фона
   {

      if(photo_state > 500)
      {
        collision = 500;// данные для графика
        stat_robot = 1; //столкновение
      }
      else
      {
         collision = 500;// данные для графика
         steering_default();// руль по умолчанию прямо
         //Serial.println("Засветка");
         stat_robot = 2;// засветка датчика препятствия
      }
  
      //robot_stop();
      Serial.print(String(background)+" ");
      Serial.print(String(photo_state)+" ");
      Serial.println(String(collision)+" ");
          
      return stat_robot;// вернуть код столкновения или ошибки
   }
   else
   {
     background = photo_state;//по умолчанию ничего не отражается от поверхности
   }
   return 0;//нет столкновения
}

int check_rotation(int count) //count - количество повторов проверки на вращение колеса
{
    int val, val2 =0;// уровни на датчиках холла
    int move_on = 0;// вращение колес происходит: +1 вперед, -1 назад, 0 - нет движения
    int i = 0;
    //Повторять проверку указанное число раз, пока не появится статус 1 или -1
    //иначе - колесо не вращается уже count времени
    
    for(i=count;i>0;i-=1)
    {
      delay(5);
      //Датчики движения работают независимо от того, включено движение или нет
      val = digitalRead(wheel);//читаем данные
      val2 = digitalRead(wheel2);//читаем данные
      
      
  
      if(val == 1) //если значение равно 1 - сигнал, не менять дальше статус, пока не придет ноль
      {
        if(state == 0)
        {
          state = 1;
      
          if(val==1 && val2==1 )
          {
             //Serial.println("Forward WH1:"+String(val)+" WH2:"+String(val2));
             move_on = 1;
          }
      
          if(val==1 && val2==0 )
          {
             //Serial.println("Backward WH1:"+String(val)+" WH2:"+String(val2));
             move_on = -1;
          }
      
        }
      }
      else
      {
        if(state == 1)
        {
          state = 0;
          
          if(val==0 && val2==0 )
          {
             //Serial.println("Forward WH1:"+String(val)+" WH2:"+String(val2));
             move_on = 1;
          }
          
          if(val==0 && val2==1 )
          {
             //Serial.println("Backward WH1:"+String(val)+" WH2:"+String(val2));
             move_on = -1;
          }
      
        }
      }
      
      //1.Глобально: объехать препятствие
      //2. Объехать и вернуться на траекторию движения
      //3. Прийти к заданной точке (база имеет площадку, подсвечиваемую светодиодами
      
      //Локально: подсвечивать и читать данные о препятствии только если робот едет, или собирается ехать. 
      //Локально: нужен транзистор, который бы отключал потребление тока через фотодиод для экономии энергии
      
      if(move_on==1 || move_on ==-1)// в движении определяем отклонение от background
      {
        //если есть движение
        return move_on;
      }
      else 
      {
         //move_on==0 
         //действие если нет движения
      }
    }
    
    //если цикл завершился и вернул управление - значит движения колеса нет:
    return 0;
}

void drive() //основной алгоритм следования к заданной точке
{
       //Serial.println("drive()");
       steering_default();// руль по центру
       robot_drive_forward();// двигаться вперед
       need_rotate = 1;// указать ожидаемое значение вращения колес
}

void loop() 
{
    //------------------- Проверка вращения колес
    int c_rot = check_rotation(100);

    
    //------------------- Проверка на столкновения:
    int collide_check = check_collision();//читаем данные о препятствии
    
    if(collide_check==0)
    {
       background_prev_ok = background;//храним пердыдущее удачное значение фона
       
       if(c_rot == need_rotate)
       {
          //если колесо вращается не туда или стоит на месте 
          //(исходя из ожидаемого статуса need_rotate  в функции drive() )
          //Serial.println("collide_check=="+String(collide_check)+", c_rot == "+String(c_rot)+", need_rotate == "+String(need_rotate)+" DRIVE");
          drive();// алгоритм следования к пункту назначения
       }
       else
       {
          // если есть ошибки
          // если колесо неожиданно перестало вращаться в движении, после команды двигаться
          
          // алголритм обхода препятствия, освобождение от застревания
          // после успешного маневра и всех проверок повторно - вернуть управление drive()
          //Serial.println("collide_check==0 No rotate");
          robot_stop();//остановить робота
       }
    }
    else
    {
      // явное препятствие
      robot_stop();//остановить робота
      
      if(collide_check==1)// действия при обнаружении препятствия на пути
      {
         //background = background_prev_ok;// назначить нормальный уровень фона до столкновения
         need_rotate = -1;// ожидается откат назад
         //robot_drive_backward();
         //Serial.println("collide_check==1 RDBL");
         //robot_drive_backward_left();
      }
      
      if(collide_check==-1)//действия при засветке
      {
          //опустить шторку фильтра (серва)
      }
      
    }
}
