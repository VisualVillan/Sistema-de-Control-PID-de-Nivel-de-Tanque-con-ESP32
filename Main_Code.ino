// --------- Pines (ESP32 + TB6612FNG) ---------
const int PWMA = 23;   
const int AIN1 = 25;   
const int AIN2 = 26;   
const int STBY = 27;   
const int TRIG = 32;   
const int ECHO = 33;   

double dt, last_time;
double integral, previous, output = 0;
double kp, ki, kd;
double setpoint = 150;

float distancia;
float altura;

// --- Parámetros del filtro robusto ---
const int   N_SAMPLES   = 40;     // ráfaga por ciclo
const int   TIMEOUT_US  = 30000;  // 30 ms
const float MIN_CM      = 1.5;    // rango válido (ajusta a tu tanque)
const float MAX_CM      = 200.0;
const float MAD_K       = 2.5;    // umbral de outliers
const float TRIM_FRAC   = 0.10;   // 10% media recortada
const float EMA_ALPHA   = 0.30;   // suavizado
const float MAX_DELTA   = 0.50;   // cm por ciclo

float distancia_filtrada = 14.75; // estado inicial para EMA

void setup()
{
  kp = 65;
  ki = 0;
  kd = 100;
  last_time = 0;

  Serial.begin(115200);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(TRIG, LOW);
  analogWrite(PWMA, 0);
  
  for(int i = 0; i < 50; i++)
  {
    Serial.println(setpoint);
    delay(100);
  }
  delay(100);
}

void loop()
{
  if (Serial.available()) {
    int c = Serial.peek();
    if (c=='\r' || c=='\n') { Serial.read(); }  // descarta line endings
    else {
      setpoint = Serial.parseFloat();
      Serial.print("Nuevo setpoint: ");
      Serial.println(setpoint);
    }
  }

  double now = millis();
  dt = (now - last_time)/1000.00;
  last_time = now;

  // 1) Distancia robusta (cruda, cm)
  float d_cruda = medirDistanciaRobusta();


  /*
  // 2) Distancia calibrada via regresión:
  float d_cal = d_cruda;
  if (d_cruda > 0.01f) {                    
    d_cal = 0.9773f*(d_cruda) + 0.2722f;    
  }
  if (d_cal < 0) d_cal = 0;                     // recorte a 0 cm
  */

  // 3) Asigna distancia final y calcula altura
  distancia = d_cruda;
  altura = 14.75f - distancia;
  float volumen_crudo = altura * 50.0f;
  float volumen = 1.0001f * volumen_crudo + 19.985f + 10.0f;

  delay(50); // periodo entre ráfagas

  // 4) Medidas de seguridad
  if (altura < 0) altura = 0;
  if (altura > 14.75) altura = 14.75;
  if (altura > 12.0) {
    output = 0;
  } else {
    double error = setpoint - volumen;
    output = pid(error);
  }
  output = constrain(output, 150, 255);
  if (output > 0 && output < 120) { // 120 es tu PWM mínimo
    output = 120;
  }
  analogWrite(PWMA, (int)output);

  // ----- Salida para Serial Plotter -----
  // Formato: setpoint altura Ymin Ymax
  Serial.print(setpoint); Serial.print(" ");
  Serial.print(volumen);   Serial.print(" ");
  Serial.print(0);        Serial.print(" "); // mínimo eje Y
  Serial.println(700);                         // máximo eje Y
  delay(100);
}

double pid(double error)
{
  static double i = 0.0;    // estado integral
  static double prev = 0.0; // error previo para derivada

  // Derivada robusta ante dt pequeño
  double DT = (dt > 1e-3) ? dt : 1e-3;
  double d  = (error - prev) / DT;

  // Salida "ideal" antes de saturar (para back-calculation)
  double u_pre = kp*error + ki*i + kd*d;

  // Saturaciones reales del actuador
  const double UMIN   = 0.0;
  const double UMAX   = 255.0;
  const double MINPWM = 120.0; // umbral de bomba

  // u_sat: salida limitada usada para corregir la integral
  double u_sat = u_pre;
  if (u_sat > UMAX) u_sat = UMAX;
  if (u_sat < UMIN) u_sat = UMIN;
  if (u_sat > 0 && u_sat < MINPWM) u_sat = MINPWM;

  // Anti-windup por back-calculation
  const double Kaw = 0.6; // 0.3–1.0 va bien
  if (ki > 1e-9) {
    i += error*DT + (u_sat - u_pre) * (Kaw/ki);
  } else {
    i += error*DT;
  }

  // Límite a la contribución integral (en unidades de PWM)
  const double Imax = 150.0; // tope del aporte integral
  double Ii = ki * i;
  if (ki > 1e-9) {
    if (Ii >  Imax) { Ii =  Imax; i =  Imax/ki; }
    if (Ii < -Imax) { Ii = -Imax; i = -Imax/ki; }
  } else {
    Ii = 0.0;
  }

  prev = error;

  // Salida final y saturación real
  double u = kp*error + Ii + kd*d;
  if (u > UMAX) u = UMAX;
  if (u < UMIN) u = UMIN;
  if (u > 0 && u < MINPWM) u = MINPWM;

  return u;
}

float medirDistanciaRobusta(){
  // 0.0343 cm/us ≈ velocidad del sonido a ~20–25°C
  const float CM_PER_US = 0.0343;

  float vals[N_SAMPLES];
  int k = 0;

  // 1) Recolectar N lecturas válidas
  for (int i = 0; i < N_SAMPLES; i++) {
    // Trigger
    digitalWrite(TRIG, LOW);
    delayMicroseconds(3);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    long t = pulseIn(ECHO, HIGH, TIMEOUT_US); // us
    if (t > 0) {
      float d = (t * CM_PER_US) / 2.0; // ida y vuelta -> /2
      if (d >= MIN_CM && d <= MAX_CM) vals[k++] = d;
    }
    delay(5); // pequeña pausa para reducir ecos múltiples
  }

  // Si hay muy pocas válidas, devuelve el último valor estable
  if (k < 5) return distancia_filtrada;

  // 2) Ordenar (inserción; k<=40)
  float sorted[N_SAMPLES];
  for (int i = 0; i < k; i++) sorted[i] = vals[i];
  for (int i = 1; i < k; i++) {
    float key = sorted[i]; int j = i - 1;
    while (j >= 0 && sorted[j] > key) { sorted[j+1] = sorted[j]; j--; }
    sorted[j+1] = key;
  }

  // Mediana
  float med = (k % 2) ? sorted[k/2] : 0.5f*(sorted[k/2-1] + sorted[k/2]);

  // 3) MAD (desviación absoluta mediana)
  float devs[N_SAMPLES];
  for (int i = 0; i < k; i++) {
    float a = vals[i] - med; devs[i] = (a < 0) ? -a : a;
  }
  for (int i = 1; i < k; i++) {
    float key = devs[i]; int j = i - 1;
    while (j >= 0 && devs[j] > key) { devs[j+1] = devs[j]; j--; }
    devs[j+1] = key;
  }
  float mad = (k % 2) ? devs[k/2] : 0.5f*(devs[k/2-1] + devs[k/2]);
  float sigma = (mad > 0) ? 1.4826f * mad : 0.5f; // robust scale
  float thr = MAD_K * sigma;

  // 4) Filtrar outliers |x - med| <= thr
  float kept[N_SAMPLES]; int m = 0;
  for (int i = 0; i < k; i++) {
    float a = vals[i] - med; if (a < 0) a = -a;
    if (a <= thr) kept[m++] = vals[i];
  }
  if (m < 5) { // si filtraste demasiado, usa todas las ordenadas
    for (int i = 0; i < k; i++) kept[i] = sorted[i];
    m = k;
  }

  // 5) Media recortada (10%)
  for (int i = 1; i < m; i++) {
    float key = kept[i]; int j = i - 1;
    while (j >= 0 && kept[j] > key) { kept[j+1] = kept[j]; j--; }
    kept[j+1] = key;
  }
  int trim = (int)(TRIM_FRAC * m);
  if (trim*2 >= m) trim = (m > 4) ? 1 : 0;
  float sum = 0; int count = 0;
  for (int i = trim; i < m - trim; i++) { sum += kept[i]; count++; }
  float trimmed_mean = (count > 0) ? (sum / count) : med;

  // 6) EMA + límite de cambio por ciclo
  float ema = EMA_ALPHA * trimmed_mean + (1.0f - EMA_ALPHA) * distancia_filtrada;
  float delta = ema - distancia_filtrada;
  if (delta >  MAX_DELTA) ema = distancia_filtrada + MAX_DELTA;
  if (delta < -MAX_DELTA) ema = distancia_filtrada - MAX_DELTA;

  distancia_filtrada = ema;
  return distancia_filtrada;

}
