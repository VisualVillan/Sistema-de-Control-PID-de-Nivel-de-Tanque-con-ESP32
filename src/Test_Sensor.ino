#include <math.h>

const int TRIG = 32;
const int ECHO = 33;

float distancia;       
float altura;

const int   N_SAMPLES   = 40;     // ráfaga por ciclo
const int   TIMEOUT_US  = 30000;  // 30 ms
const float MIN_CM      = 1.5;    // rango válido (ajusta a tu tanque)
const float MAX_CM      = 200.0;
const float MAD_K       = 2.5;    // umbral de outliers
const float TRIM_FRAC   = 0.10;   // 10% media recortada
const float EMA_ALPHA   = 0.30;   // suavizado
const float MAX_DELTA   = 0.50;   // cm por ciclo

float distancia_filtrada = 14.75; // estado inicial para EMA

void setup() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIG, LOW);
  Serial.begin(9600);
}

void loop() {
  // 1) Distancia robusta (cruda, cm)
  float d_cruda = medirDistanciaRobusta();

  // 2) Distancia calibrada via regresión: y = 4.365*ln(x) - 1.5146
  float d_cal = d_cruda;
  if (d_cruda > 0.01f) {                        // evita log(0)
    d_cal = 0.9773f*(d_cruda) + 0.2722f;    // ln() natural
  }
  if (d_cal < 0) d_cal = 0;                     // recorte a 0 cm

  // 3) Asigna distancia final y calcula altura
  distancia = d_cal;
  altura = 14.75f - distancia;
  if (altura < 0) altura = 0;

  // Log útil
  Serial.print("Cruda(cm)=");    Serial.print(d_cruda, 2);
  Serial.print("  Cal(cm)=");    Serial.print(distancia, 2);
  Serial.print("  Altura(cm)="); Serial.println(altura, 2);

  delay(200); 
}

// Función de medición robusta 
float medirDistanciaRobusta() {
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
    delay(5);
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
  float sigma = (mad > 0) ? 1.4826f * mad : 0.5f; 
  float thr = MAD_K * sigma;

  // 4) Filtrar outliers |x - med| <= thr
  float kept[N_SAMPLES]; int m = 0;
  for (int i = 0; i < k; i++) {
    float a = vals[i] - med; if (a < 0) a = -a;
    if (a <= thr) kept[m++] = vals[i];
  }
  if (m < 5) { 
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
