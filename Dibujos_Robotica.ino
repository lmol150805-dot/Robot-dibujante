#include <ESP32Servo.h>
#include <math.h>

const int N = 3;
const int PINES[N] = {13, 26, 2};   // servo 1 (base), servo 2 (codo), servo 3 (lapiz)
const int PULSO_MIN = 500;          // us para 0 grados
const int PULSO_MAX = 2400;         // us para 180 grados

unsigned long tPaso = 30;           // ms por grado en el modo de prueba

Servo servos[N];
int angulo[N]   = {90, 90, 0};      // posicion actual (lapiz arriba en 0)
int objetivo[N] = {90, 90, 0};      // posicion a la que va
bool barrido[N] = {false, false, false};
unsigned long tAnterior = 0;

// ================= MODELO DEL ROBOT (cm y grados) =================
// Valores calibrados con el comando cal (12 puntos medidos, error rms 0.6 mm)
const float L1   = 10.22;           // base -> codo
const float L2   = 9.91;            // codo -> motor del lapiz
const float DESV = 3.15;            // el lapiz queda corrido hacia la hoja
const float SENTIDO[2] = {1.0373, 0.9956};  // grados de servo por grado real (la base gira 3.6% menos)
const float Q_EN_90[2] = {-0.02, 0.89};     // angulo real de cada articulacion con el servo en 90
const float CODO = -1;              // el codo dobla hacia la hoja (servo 2 por debajo de 90)

const int LAPIZ_ABAJO  = 90;        // lapiz tocando la hoja
const int LAPIZ_ARRIBA = 0;         // lapiz levantado

// ================= CUADRADO =================
const float CENTRO_S1 = 55, CENTRO_S2 = 40;   // angulos de los servos en el centro del cuadrado
const float S_MIN = 0, S_MAX = 90;  // rango permitido de los servos 1 y 2
const float LADO_X = 4.0;           // lados A -> B y C -> D
const float LADO_Y = 4.0;           // lados B -> C y D -> A
const float DIR_X = -1;             // hacia donde va el lado A -> B (1 o -1)
const float DIR_Y = 1;              // hacia donde va el lado B -> C (1 o -1)

// ================= TRAYECTORIA =================
int   tMuestraUsuario = 100;        // ms entre muestras (comando tm). Lento = menos vibracion
float pasoMin = 1.0;                // grados que avanza cada servo por muestra (comando pm, en decimas)
const float ACEL = 0.3;             // fraccion del lado para acelerar y para frenar (perfil trapezoidal)
const float GIRO_MIN = 10.0;        // si un servo gira menos que esto en un lado, se toma como quieto
const int   PAUSA_ESQUINA = 250;    // ms quieto en cada esquina
const int   PAUSA_LAPIZ = 500;      // ms de espera despues de subir o bajar el lapiz (que llegue de verdad)
const float UMBRAL_LENTO = 0.7;     // si un servo avanza muy poquito por muestra, se le manda solo cada vez que acumule esto
const float HOLGURA[2] = {1.5, 1.0}; // juego de los engranajes de los servos 1 y 2 [grados]
const int MAX_PTS = 40;             // puntos maximos de una figura
bool suave = false;                 // figura curva (circulo): sin pausas ni frenadas en cada punto
// Figuras: 0 cuadrado, 1 casa, 2 triangulo, 3 rectangulo, 4 rombo, 5 estrella, 6 hexagono, 7 circulo
const char* NOMBRE_FIG[] = {"cuadrado", "casa", "triangulo", "rectangulo", "rombo", "estrella", "hexagono", "circulo"};
int figura = 0;
int extraMM = 2;                    // mm que se pasa cada linea en las esquinas para que la figura cierre (comando ex)
int extraFinalMM = 6;               // mm extra solo para la ultima linea, que no tiene otra que la cruce (comando exf)
int muestras[MAX_PTS];              // muestras de cada tramo (se calculan solas)
int tMuestra[MAX_PTS];              // ms entre muestras de cada tramo

// ================= MAQUINA DE ESTADOS =================
enum Estado { REPOSO, SUBIR_LAPIZ, IR_INICIO, BAJAR_LAPIZ, DIBUJAR, ESQUINA, SUBIR_FINAL,
              CAL_SUBIR, CAL_MOVER, CAL_ESPERA, CAL_BAJAR, CAL_MARCAR };
Estado fsm = REPOSO;
const char* NOMBRE_ESTADO[] = {"REPOSO", "SUBIR_LAPIZ", "IR_INICIO", "BAJAR_LAPIZ", "DIBUJAR", "ESQUINA", "SUBIR_FINAL",
                               "CAL_SUBIR", "CAL_MOVER", "CAL_ESPERA", "CAL_BAJAR", "CAL_MARCAR"};

float angF[N] = {90, 90, 0};        // angulo de servo con decimales
int sentidoMov[2] = {0, 0};         // ultimo sentido de giro de los servos 1 y 2 (para la holgura)
unsigned long tEstado = 0;
float ex[MAX_PTS], ey[MAX_PTS];     // puntos de la figura (se recorren en orden con el lapiz abajo)
int nTramos = 4;                    // tramos de la figura
int ladoTecho = 2;                  // lado del cuadrado donde va el techo: 1=AB 2=BC 3=CD 4=DA (comando techo)
const float ALTO_TECHO = 3.0;       // altura del triangulo del techo [cm]
int lado = 0, muestra = 0;
float ini1, ini2, fin1, fin2;       // para ir a la esquina A
int pasosIr = 0, kIr = 0;
float q[2];                         // angulos articulares del modelo [rad]
bool detenido = false;
unsigned long tLapiz = 0;           // momento en que el lapiz termino de moverse
bool lento1[MAX_PTS], lento2[MAX_PTS];   // servo que en ese tramo avanza menos que su zona muerta por muestra
float ult1, ult2;                   // ultimo angulo enviado a cada servo

// ================= CALIBRACION (comando cal) =================
// El robot marca un punto en cada una de estas posiciones de los servos.
// Se mide (x, y) de cada punto en la hoja y con eso se ajusta el modelo.
const int NCAL = 12;
const float CAL_S1[NCAL] = {40, 40, 40, 40, 55, 55, 55, 55, 70, 70, 70, 70};
const float CAL_S2[NCAL] = {20, 40, 60, 75, 75, 60, 40, 20, 20, 40, 60, 75};
const float CAL_APROX = 4;          // llega a cada punto siempre desde abajo (mismo lado del juego)
bool calibrando = false;
int iCal = 0, faseCal = 0;

// Declaracion de funciones
void menu();
bool aEntero(String t, int &v);
void mover(int i, int a);
void iniciarBarrido(int i);
void estado();
void invalido();
void escribir(int i, float a);
void servoAArticular(float s1, float s2, float &q1, float &q2);
void articularAServo(float q1, float q2, float &s1, float &s2);
void cinDirecta(float q1, float q2, float &x, float &y);
void jacobiano(float q1, float q2, float J[2][2]);
bool cinInversaJacobiano(float xd, float yd, float &q1, float &q2);
bool cinInversaGeometrica(float x, float y, float &s1, float &s2);
float perfil(float u);
float codoCompensado(float s1, float x, float y);
void puntoTrayectoria(int l, int m, float &x, float &y);
bool prepararFigura(float c1, float c2, int tipo);
void iniciarDibujo(float c1, float c2, int tipo);
void poligono(float cx, float cy, int lados, float radio, float giro, int salto);
void cambiarEstado(Estado e);
void actualizarFSM();
bool moverLapizPaso(int destino);
void procesar(String cmd);
void iniciarMovimiento(float t1, float t2);
void pruebaSimultaneo();

void setup() {
  Serial.begin(115200);
  delay(1000);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < N; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(PINES[i], PULSO_MIN, PULSO_MAX);
    servos[i].write(angulo[i]);
    delay(300);  // arranque escalonado para no tumbar la fuente
  }
  menu();
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) procesar(cmd);
  }

  actualizarFSM();

  if (millis() - tAnterior >= tPaso) {
    tAnterior = millis();
    for (int i = 0; i < N; i++) {
      if (angulo[i] < objetivo[i]) {
        angulo[i]++;
        angF[i] = angulo[i];
        servos[i].write(angulo[i]);
      } else if (angulo[i] > objetivo[i]) {
        angulo[i]--;
        angF[i] = angulo[i];
        servos[i].write(angulo[i]);
      } else if (barrido[i]) {
        objetivo[i] = (angulo[i] >= 180) ? 0 : 180;
      }
    }
  }
}

void menu() {
  Serial.println("Robot dibujante - ESP32 WROOM-32 + 3 MG996R");
  Serial.println("Comandos:");
  Serial.println("  d     : dibujar el cuadrado (p para detener)");
  Serial.println("  dc    : dibujar la ultima figura centrada donde esta el lapiz ahora");
  Serial.println("  h     : dibujar la casa (cuadrado + techo)");
  Serial.println("  f n   : figura 0 cuadrado, 1 casa, 2 triangulo, 3 rectangulo, 4 rombo, 5 estrella, 6 hexagono, 7 circulo");
  Serial.println("  techo n : lado del techo 1=AB 2=BC 3=CD 4=DA. Ej: techo 2");
  Serial.println("  ex mm : cuanto se pasa cada linea en las esquinas (0-6). Ej: ex 3");
  Serial.println("  exf mm: cuanto mas se pasa la ultima linea (0-15). Ej: exf 8");
  Serial.println("  cal   : calibracion, marca 12 puntos para medir");
  Serial.println("  ts    : prueba si los servos 1 y 2 se mueven al mismo tiempo");
  Serial.println("  tm ms : tiempo entre muestras al dibujar (20-500). Ej: tm 100");
  Serial.println("  pm d  : grados por muestra en decimas (3-50). Ej: pm 15 = 1.5 grados");
  Serial.println("  n ang : servo n (1-3) al angulo ang (0-180). Ej: 2 45");
  Serial.println("  a ang : los 3 servos al angulo ang. Ej: a 120");
  Serial.println("  s     : barrido de los 3");
  Serial.println("  sn    : barrido solo del servo n. Ej: s3");
  Serial.println("  p     : parar todos donde esten");
  Serial.println("  c     : servos 1 y 2 en 90 y lapiz arriba (0)");
  Serial.println("  v ms  : velocidad, ms por grado (1-200). Ej: v 50");
  Serial.println("  e     : ver estado");
  Serial.printf("Velocidad actual: %lu ms por grado\n", tPaso);
}

bool aEntero(String t, int &v) {
  t.trim();
  if (t.length() == 0) return false;
  for (unsigned int i = 0; i < t.length(); i++) {
    if (!isDigit(t[i])) return false;
  }
  v = t.toInt();
  return true;
}

void mover(int i, int a) {
  barrido[i] = false;
  objetivo[i] = a;
  Serial.printf("Servo %d -> %d grados\n", i + 1, a);
}

void iniciarBarrido(int i) {
  barrido[i] = true;
  objetivo[i] = (angulo[i] <= 90) ? 180 : 0;
}

void estado() {
  Serial.printf("Estado: %s\n", NOMBRE_ESTADO[fsm]);
  Serial.printf("Velocidad: %lu ms por grado (%.1f grados/s)\n", tPaso, 1000.0 / tPaso);
  for (int i = 0; i < N; i++) {
    Serial.printf("Servo %d | GPIO %d | actual %d | objetivo %d | barrido %s\n",
                  i + 1, PINES[i], angulo[i], objetivo[i], barrido[i] ? "si" : "no");
  }
  Serial.printf("Trayectoria: tm = %d ms por muestra | pm = %.1f grados por muestra\n", tMuestraUsuario, pasoMin);
}

void invalido() {
  Serial.println("Comando no valido");
}

// Escribe un angulo con decimales y compensa la holgura de los engranajes
void escribir(int i, float a) {
  if (a < 0) a = 0;
  if (a > 180) a = 180;
  float salida = a;
  if (i < 2 && !calibrando) {           // en calibracion se mandan los angulos puros
    if (a > angF[i] + 0.01) sentidoMov[i] = 1;
    if (a < angF[i] - 0.01) sentidoMov[i] = -1;
    salida = a + sentidoMov[i] * HOLGURA[i] / 2;
  }
  angF[i] = a;
  angulo[i] = (int)round(a);
  objetivo[i] = angulo[i];
  servos[i].writeMicroseconds(PULSO_MIN + salida * (PULSO_MAX - PULSO_MIN) / 180.0);
}

// ================= CINEMATICA =================

// Angulo de servo [grados] <-> angulo articular del modelo [rad]
void servoAArticular(float s1, float s2, float &q1, float &q2) {
  q1 = ((s1 - 90) / SENTIDO[0] + Q_EN_90[0]) * PI / 180.0;
  q2 = ((s2 - 90) / SENTIDO[1] + Q_EN_90[1]) * PI / 180.0;
}

void articularAServo(float q1, float q2, float &s1, float &s2) {
  s1 = 90 + SENTIDO[0] * (q1 * 180.0 / PI - Q_EN_90[0]);
  s2 = 90 + SENTIDO[1] * (q2 * 180.0 / PI - Q_EN_90[1]);
}

// Cinematica directa: (q1, q2) -> posicion del lapiz (x, y)
void cinDirecta(float q1, float q2, float &x, float &y) {
  x = L1 * cos(q1) + L2 * cos(q1 + q2) + DESV * sin(q1 + q2);
  y = L1 * sin(q1) + L2 * sin(q1 + q2) - DESV * cos(q1 + q2);
}

// Jacobiano: derivada de (x, y) respecto a (q1, q2)
void jacobiano(float q1, float q2, float J[2][2]) {
  float s1 = sin(q1), c1 = cos(q1), s12 = sin(q1 + q2), c12 = cos(q1 + q2);
  J[0][0] = -L1 * s1 - L2 * s12 + DESV * c12;
  J[0][1] =           - L2 * s12 + DESV * c12;
  J[1][0] =  L1 * c1 + L2 * c12 + DESV * s12;
  J[1][1] =            L2 * c12 + DESV * s12;
}

// Cinematica inversa con el Jacobiano (como en clase): q(k+1) = q(k) + J^-1 * [X(k+1) - X(k)]
// X(k) se calcula con la cinematica directa, asi el error no se acumula
bool cinInversaJacobiano(float xd, float yd, float &q1, float &q2) {
  for (int it = 0; it < 3; it++) {
    float x, y, J[2][2];
    cinDirecta(q1, q2, x, y);
    jacobiano(q1, q2, J);
    float det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    if (fabs(det) < 1e-3) return false;               // singularidad (brazo estirado)
    float dx = xd - x, dy = yd - y;
    q1 += ( J[1][1] * dx - J[0][1] * dy) / det;        // J^-1 * dX
    q2 += (-J[1][0] * dx + J[0][0] * dy) / det;
  }
  return true;
}

// Cinematica inversa geometrica (solo para revisar el alcance antes de dibujar)
bool cinInversaGeometrica(float x, float y, float &s1, float &s2) {
  float Le = sqrt(L2 * L2 + DESV * DESV);
  float delta = -atan2(DESV, L2);
  float c = (x * x + y * y - L1 * L1 - Le * Le) / (2 * L1 * Le);
  if (c < -1 || c > 1) return false;
  float fi = CODO * acos(c);
  float q1 = atan2(y, x) - atan2(Le * sin(fi), L1 + Le * cos(fi));
  float q2 = fi - delta;
  articularAServo(q1, q2, s1, s2);
  if (s1 < S_MIN || s1 > S_MAX || s2 < S_MIN || s2 > S_MAX) return false;
  return true;
}

// ================= TRAYECTORIA (interpolacion lineal en el espacio cartesiano) =================

// Punto deseado del lado l en la muestra m (velocidad constante sobre la linea)
// Perfil de velocidad trapezoidal: acelera suave, velocidad constante y frena suave
// u = tiempo normalizado (0 a 1), devuelve la fraccion del lado recorrida
// Con la base quieta en s1, el lapiz solo puede moverse en un circulo alrededor del codo.
// Devuelve el angulo del servo del codo que deja el lapiz lo mas cerca posible de (x, y)
float codoCompensado(float s1, float x, float y) {
  float q1, qBasura, sBasura, s2;
  servoAArticular(s1, 90, q1, qBasura);
  float delta = -atan2(DESV, L2);
  float exC = L1 * cos(q1), eyC = L1 * sin(q1);          // posicion del codo
  float q2 = atan2(y - eyC, x - exC) - q1 - delta;
  while (q2 >  PI) q2 -= 2 * PI;
  while (q2 < -PI) q2 += 2 * PI;
  articularAServo(q1, q2, sBasura, s2);
  return s2;
}

float perfil(float u) {
  if (u >= 1) return 1;
  if (suave) return u;                              // curva: velocidad constante, sin frenar en cada punto
  float v = 1.0 / (1.0 - ACEL);                     // velocidad maxima
  if (u < ACEL)     return v * u * u / (2 * ACEL);
  if (u < 1 - ACEL) return v * (u - ACEL / 2);
  return 1 - v * (1 - u) * (1 - u) / (2 * ACEL);
}

void puntoTrayectoria(int l, int m, float &x, float &y) {
  float s = perfil((float)m / muestras[l]);
  x = ex[l] + (ex[l + 1] - ex[l]) * s;
  y = ey[l] + (ey[l + 1] - ey[l]) * s;
}

// Arma la lista de puntos de la figura: cuadrado, o casa (cuadrado + triangulo encima de un lado)
// Poligono regular (o estrella si salto = 2) alrededor de (cx, cy). giro en grados
void poligono(float cx, float cy, int lados, float radio, float giro, int salto) {
  for (int i = 0; i <= lados; i++) {
    float a = (giro + 360.0 * i * salto / lados) * PI / 180.0;
    ex[i] = cx + radio * cos(a);
    ey[i] = cy + radio * sin(a);
  }
  nTramos = lados;
}

bool prepararFigura(float c1, float c2, int tipo) {
  bool casa = (tipo == 1);
  float qc1, qc2, cx, cy;
  servoAArticular(c1, c2, qc1, qc2);
  cinDirecta(qc1, qc2, cx, cy);                     // centro del cuadrado
  float AX = cx - DIR_X * LADO_X / 2;
  float AY = cy - DIR_Y * LADO_Y / 2;
  float px[4] = {AX, AX + DIR_X * LADO_X, AX + DIR_X * LADO_X, AX};   // esquinas A B C D
  float py[4] = {AY, AY, AY + DIR_Y * LADO_Y, AY + DIR_Y * LADO_Y};

  if (!casa) {
    for (int i = 0; i < 5; i++) { ex[i] = px[i % 4];  ey[i] = py[i % 4]; }
    nTramos = 4;
  } else {
    // El cuadrado empieza en la esquina final del lado del techo, da la vuelta completa
    // y sigue sin levantar el lapiz: esquina -> punta del techo -> otra esquina
    int k = ladoTecho - 1;                          // lado k va de la esquina k a la k+1
    for (int i = 0; i < 5; i++) { ex[i] = px[(k + 1 + i) % 4];  ey[i] = py[(k + 1 + i) % 4]; }
    float mx = (px[k] + px[(k + 1) % 4]) / 2, my = (py[k] + py[(k + 1) % 4]) / 2;   // mitad del lado
    float nx = mx - cx, ny = my - cy;               // hacia afuera del cuadrado
    float d = sqrt(nx * nx + ny * ny);
    ex[5] = mx + ALTO_TECHO * nx / d;  ey[5] = my + ALTO_TECHO * ny / d;            // punta del techo
    ex[6] = px[k];  ey[6] = py[k];
    nTramos = 6;
  }

  // Otras figuras, todas alrededor del mismo centro (cx, cy) [cm]
  suave = false;
  if (tipo == 2) poligono(cx, cy, 3, 2.3, 90, 1);          // triangulo equilatero de 4 cm de lado
  if (tipo == 3) {                                          // rectangulo 5 x 3 cm
    float rx[5] = {cx + 2.5f, cx - 2.5f, cx - 2.5f, cx + 2.5f, cx + 2.5f};
    float ry[5] = {cy - 1.5f, cy - 1.5f, cy + 1.5f, cy + 1.5f, cy - 1.5f};
    for (int i = 0; i < 5; i++) { ex[i] = rx[i];  ey[i] = ry[i]; }
    nTramos = 4;
  }
  if (tipo == 4) poligono(cx, cy, 4, 2.3, 0, 1);           // rombo
  if (tipo == 5) poligono(cx, cy, 5, 2.5, 90, 2);          // estrella de 5 puntas, de un solo trazo
  if (tipo == 6) poligono(cx, cy, 6, 2.2, 0, 1);           // hexagono
  if (tipo == 7) { poligono(cx, cy, 36, 2.0, 0, 1);  suave = true; }   // circulo de 4 cm de diametro

  // Cada linea se alarga extraMM por los dos extremos, asi las esquinas se cruzan y la figura cierra.
  // Entre una linea y la siguiente queda un tramo corto que une el final de una con el inicio de la otra.
  if (extraMM > 0 && !suave) {
    float ox[MAX_PTS], oy[MAX_PTS];
    int n = nTramos;
    for (int i = 0; i <= n; i++) { ox[i] = ex[i];  oy[i] = ey[i]; }
    for (int i = 0; i < n; i++) {
      float dx = ox[i + 1] - ox[i], dy = oy[i + 1] - oy[i];
      float L = sqrt(dx * dx + dy * dy);
      float ux = dx / L * extraMM / 10.0, uy = dy / L * extraMM / 10.0;
      ex[2 * i]     = ox[i] - ux;      ey[2 * i]     = oy[i] - uy;       // inicio un poco antes
      ex[2 * i + 1] = ox[i + 1] + ux;  ey[2 * i + 1] = oy[i + 1] + uy;   // final un poco despues
    }
    nTramos = 2 * n - 1;
  }

  // La ultima linea se alarga un poco mas, para que llegue a la esquina final.
  // Si con eso se sale del alcance de los servos, se deja sin alargar.
  if (extraFinalMM > 0 && !suave) {
    int u = nTramos;
    float dx = ex[u] - ex[u - 1], dy = ey[u] - ey[u - 1];
    float L = sqrt(dx * dx + dy * dy);
    float nx = ex[u] + dx / L * extraFinalMM / 10.0;
    float ny = ey[u] + dy / L * extraFinalMM / 10.0;
    float t1, t2;
    if (cinInversaGeometrica(nx, ny, t1, t2)) { ex[u] = nx;  ey[u] = ny; }
    else Serial.println("La ultima linea no se alarga: se saldria del alcance");
  }

  // Simulacion de todo el recorrido con el Jacobiano antes de mover el robot
  float s1, s2, x, y;
  if (!cinInversaGeometrica(ex[0], ey[0], s1, s2)) {
    Serial.println("Punto inicial fuera de alcance. No se dibuja.");
    return false;
  }
  float a1 = s1, a2 = s2;
  servoAArticular(s1, s2, qc1, qc2);
  for (int l = 0; l < nTramos; l++) {
    // Cuanto gira cada servo en el lado -> numero de muestras para que los DOS se muevan
    // por encima de su zona muerta en cada muestra (asi se mueven al tiempo)
    float g1 = 0, g2 = 0, b1, b2, p1 = a1, p2 = a2;
    for (int m = 1; m <= 50; m++) {
      float u = m / 50.0;
      if (!cinInversaGeometrica(ex[l] + (ex[l + 1] - ex[l]) * u, ey[l] + (ey[l + 1] - ey[l]) * u, b1, b2)) {
        Serial.printf("Tramo %d fuera de alcance (servos entre %.0f y %.0f). No se dibuja.\n", l + 1, S_MIN, S_MAX);
        return false;
      }
      g1 += fabs(b1 - p1);  g2 += fabs(b2 - p2);
      p1 = b1;  p2 = b2;
    }
    float menor = fmin(g1, g2), mayor = fmax(g1, g2);
    if (menor >= GIRO_MIN) muestras[l] = (int)round(menor / pasoMin);
    else                   muestras[l] = (int)round(mayor / pasoMin);
    muestras[l] = (int)round(muestras[l] / (1.0 - ACEL));   // mas muestras: en el tramo rapido el paso sigue igual
    if (muestras[l] < 1) muestras[l] = 1;
    tMuestra[l] = tMuestraUsuario;
    // Un servo que gira poco en el tramo recibiria pasitos que no alcanza a hacer (zona muerta):
    // a ese se le manda el angulo solo cuando acumula UMBRAL_LENTO, para que si llegue a su destino
    lento1[l] = (g1 / muestras[l] < 0.5 && g1 > 0.3);
    lento2[l] = (g2 / muestras[l] < 0.5 && g2 > 0.3);

    float vmax1 = 0, vmax2 = 0, vmin1 = 999, vmin2 = 999, errMax = 0;
    for (int m = 1; m <= muestras[l]; m++) {
      puntoTrayectoria(l, m, x, y);
      if (!cinInversaJacobiano(x, y, qc1, qc2)) {
        Serial.printf("Singularidad en (%.2f, %.2f). No se dibuja.\n", x, y);
        return false;
      }
      articularAServo(qc1, qc2, s1, s2);
      if (s1 < S_MIN || s1 > S_MAX || s2 < S_MIN || s2 > S_MAX) {
        Serial.printf("Punto (%.2f, %.2f) fuera del rango de los servos. No se dibuja.\n", x, y);
        return false;
      }
      float xr, yr;
      cinDirecta(qc1, qc2, xr, yr);                   // validacion con la cinematica directa
      float e = sqrt((xr - x) * (xr - x) + (yr - y) * (yr - y));
      if (e > errMax) errMax = e;
      vmax1 = fmax(vmax1, fabs(s1 - a1));  vmin1 = fmin(vmin1, fabs(s1 - a1));
      vmax2 = fmax(vmax2, fabs(s2 - a2));  vmin2 = fmin(vmin2, fabs(s2 - a2));
      a1 = s1;  a2 = s2;
    }
    Serial.printf("Tramo %d: gira %.1f / %.1f -> %d muestras cada %d ms | paso servo1 %.2f-%.2f, servo2 %.2f-%.2f grados | error FK %.4f mm\n",
                  l + 1, g1, g2, muestras[l], tMuestra[l], vmin1, vmax1, vmin2, vmax2, errMax * 10);
    Serial.printf("        este tramo tarda %.2f s\n", muestras[l] * tMuestra[l] / 1000.0);
  }
  return true;
}

// ================= MAQUINA DE ESTADOS DEL DIBUJO =================

void iniciarDibujo(float c1, float c2, int tipo) {
  if (!prepararFigura(c1, c2, tipo)) return;
  detenido = false;
  for (int i = 0; i < N; i++) { barrido[i] = false; objetivo[i] = angulo[i]; }
  Serial.printf("Dibujando %s...\n", NOMBRE_FIG[tipo]);
  cambiarEstado(SUBIR_LAPIZ);
}

// Prueba: mueve los servos 1 y 2 juntos, primero con pasos grandes y luego con pasos de 1 grado.
// Si con pasos grandes se mueven al tiempo pero con pasos de 1 grado van por turnos,
// el problema es la zona muerta del servo, no el codigo.
void pruebaSimultaneo() {
  float b1 = angF[0], b2 = angF[1];
  Serial.println("Prueba 1: pasos de 10 grados, los dos servos en la misma linea de codigo");
  for (int r = 0; r < 3; r++) {
    escribir(0, b1 + 10);  escribir(1, b2 + 10);  delay(1000);
    escribir(0, b1);       escribir(1, b2);       delay(1000);
  }
  Serial.println("Prueba 2: pasos de 1 grado cada 100 ms");
  for (int i = 1; i <= 15; i++) { escribir(0, b1 + i);  escribir(1, b2 + i);  delay(100); }
  for (int i = 14; i >= 0; i--) { escribir(0, b1 + i);  escribir(1, b2 + i);  delay(100); }
  Serial.println("Prueba terminada");
}

// Prepara un movimiento articular de los dos servos juntos hasta (t1, t2)
void iniciarMovimiento(float t1, float t2) {
  ini1 = angF[0];  ini2 = angF[1];
  fin1 = t1;  fin2 = t2;
  pasosIr = (int)ceil(fmax(fabs(fin1 - ini1), fabs(fin2 - ini2)));
  if (pasosIr < 1) pasosIr = 1;
  kIr = 0;
}

void cambiarEstado(Estado e) {
  fsm = e;
  tEstado = millis();
}

// Mueve el lapiz 1 grado. Devuelve true cuando llego
bool moverLapizPaso(int destino) {
  int a = (int)round(angF[2]);
  if (a == destino) return true;
  if (a < destino) a++; else a--;
  escribir(2, a);
  return false;
}

void actualizarFSM() {
  unsigned long ahora = millis();
  float s1, s2, x, y;

  switch (fsm) {
    case REPOSO:
      break;

    case SUBIR_LAPIZ:
      if (ahora - tEstado >= 10) {
        tEstado = ahora;
        if (moverLapizPaso(LAPIZ_ARRIBA)) {
          if (tLapiz == 0) tLapiz = ahora;
          if (ahora - tLapiz < (unsigned long)PAUSA_LAPIZ) break;   // espera a que el lapiz suba de verdad
          tLapiz = 0;
          cinInversaGeometrica(ex[0], ey[0], fin1, fin2);
          ini1 = angF[0];  ini2 = angF[1];
          pasosIr = (int)ceil(fmax(fabs(fin1 - ini1), fabs(fin2 - ini2)));
          if (pasosIr < 1) pasosIr = 1;
          kIr = 0;
          cambiarEstado(IR_INICIO);
        }
      }
      break;

    case IR_INICIO:   // los dos servos juntos hasta la esquina A
      if (ahora - tEstado >= tPaso) {
        tEstado = ahora;
        kIr++;
        escribir(0, ini1 + (fin1 - ini1) * kIr / pasosIr);
        escribir(1, ini2 + (fin2 - ini2) * kIr / pasosIr);
        if (kIr >= pasosIr) cambiarEstado(BAJAR_LAPIZ);
      }
      break;

    case BAJAR_LAPIZ:
      if (ahora - tEstado >= 10) {
        tEstado = ahora;
        if (moverLapizPaso(LAPIZ_ABAJO)) {
          if (tLapiz == 0) tLapiz = ahora;
          if (ahora - tLapiz < (unsigned long)PAUSA_LAPIZ) break;   // espera a que el lapiz toque la hoja
          tLapiz = 0;
          servoAArticular(angF[0], angF[1], q[0], q[1]);   // q inicial = donde estan los servos
          ult1 = angF[0];  ult2 = angF[1];
          lado = 0;  muestra = 0;
          Serial.println("Tramo 1");
          cambiarEstado(DIBUJAR);
        }
      }
      break;

    case DIBUJAR:     // cada muestra: punto de la linea -> Jacobiano -> los dos servos al tiempo
      if (ahora - tEstado >= (unsigned long)tMuestra[lado]) {
        tEstado = ahora;
        muestra++;
        puntoTrayectoria(lado, muestra, x, y);
        cinInversaJacobiano(x, y, q[0], q[1]);
        articularAServo(q[0], q[1], s1, s2);
        {
          bool fin = (muestra >= muestras[lado]);
          bool m1 = !lento1[lado] || fin || fabs(s1 - ult1) >= UMBRAL_LENTO;
          if (m1) { escribir(0, s1);  ult1 = s1; }      // los dos en la misma muestra
          // En curvas: si la base se queda quieta esperando, el codo corrige para seguir sobre la curva
          if (suave && !m1) s2 = codoCompensado(ult1, x, y);
          bool m2 = suave || !lento2[lado] || fin || fabs(s2 - ult2) >= UMBRAL_LENTO;
          if (m2) { escribir(1, s2);  ult2 = s2; }
        }
        if (muestra >= muestras[lado]) {
          float xr, yr;
          cinDirecta(q[0], q[1], xr, yr);
          Serial.printf("  punto %d: error FK = %.4f mm\n", lado + 2,
                        10 * sqrt((xr - ex[lado + 1]) * (xr - ex[lado + 1]) + (yr - ey[lado + 1]) * (yr - ey[lado + 1])));
          lado++;  muestra = 0;
          cambiarEstado(ESQUINA);
        }
      }
      break;

    case ESQUINA:
      if (suave || ahora - tEstado >= (unsigned long)PAUSA_ESQUINA) {
        if (lado >= nTramos) {
          cambiarEstado(SUBIR_FINAL);
        } else {
          Serial.printf("Tramo %d\n", lado + 1);
          cambiarEstado(DIBUJAR);
        }
      }
      break;

    case SUBIR_FINAL:
      if (ahora - tEstado >= 10) {
        tEstado = ahora;
        if (moverLapizPaso(LAPIZ_ARRIBA)) {
          if (detenido) Serial.println("Detenido");
          else Serial.println("Figura terminada");
          calibrando = false;
          cambiarEstado(REPOSO);
        }
      }
      break;

    // ---------- calibracion ----------
    case CAL_SUBIR:
      if (ahora - tEstado >= 10) {
        tEstado = ahora;
        if (moverLapizPaso(LAPIZ_ARRIBA)) {
          if (iCal >= NCAL) {
            calibrando = false;
            Serial.println("\nCalibracion terminada. Mide (x, y) en mm de cada punto en la hoja:");
            Serial.println("  origen en el punto 1, eje x a lo largo de la cuadricula, y perpendicular.");
            Serial.println("  Pasa la tabla: punto, x, y");
            cambiarEstado(REPOSO);
          } else {
            iniciarMovimiento(CAL_S1[iCal] - CAL_APROX, CAL_S2[iCal] - CAL_APROX);   // punto de aproximacion
            faseCal = 0;
            cambiarEstado(CAL_MOVER);
          }
        }
      }
      break;

    case CAL_MOVER:
      if (ahora - tEstado >= tPaso) {
        tEstado = ahora;
        kIr++;
        escribir(0, ini1 + (fin1 - ini1) * kIr / pasosIr);
        escribir(1, ini2 + (fin2 - ini2) * kIr / pasosIr);
        if (kIr >= pasosIr) {
          if (faseCal == 0) {
            iniciarMovimiento(CAL_S1[iCal], CAL_S2[iCal]);   // ultimo tramo, siempre subiendo
            faseCal = 1;
          } else {
            cambiarEstado(CAL_ESPERA);
          }
        }
      }
      break;

    case CAL_ESPERA:
      if (ahora - tEstado >= 500) cambiarEstado(CAL_BAJAR);
      break;

    case CAL_BAJAR:
      if (ahora - tEstado >= 10) {
        tEstado = ahora;
        if (moverLapizPaso(LAPIZ_ABAJO)) {
          Serial.printf("Punto %2d: servo1 = %.1f  servo2 = %.1f\n", iCal + 1, CAL_S1[iCal], CAL_S2[iCal]);
          cambiarEstado(CAL_MARCAR);
        }
      }
      break;

    case CAL_MARCAR:
      if (ahora - tEstado >= 400) {
        iCal++;
        cambiarEstado(CAL_SUBIR);
      }
      break;
  }
}

// ================= COMANDOS =================

void procesar(String cmd) {
  bool ocupado = (fsm != REPOSO);

  if (cmd == "p") {
    if (ocupado) {
      detenido = true;
      cambiarEstado(SUBIR_FINAL);
      return;
    }
    for (int i = 0; i < N; i++) { barrido[i] = false; objetivo[i] = angulo[i]; }
    Serial.println("Detenidos");
    estado();
    return;
  }
  if (cmd == "e") { estado(); return; }
  if (ocupado) { Serial.println("Ocupado dibujando (p para detener)"); return; }

  if (cmd == "ts") {
    pruebaSimultaneo();
  } else if (cmd == "cal") {
    calibrando = true;
    detenido = false;
    iCal = 0;
    for (int i = 0; i < N; i++) { barrido[i] = false; objetivo[i] = angulo[i]; }
    Serial.println("Calibracion: marca 12 puntos. Numeralos a lapiz a medida que aparecen.");
    cambiarEstado(CAL_SUBIR);
  } else if (cmd == "d") {
    iniciarDibujo(CENTRO_S1, CENTRO_S2, 0);
  } else if (cmd == "h") {
    iniciarDibujo(CENTRO_S1, CENTRO_S2, 1);
  } else if (cmd == "dc") {
    iniciarDibujo(angF[0], angF[1], figura);
  } else if (cmd == "s") {
    for (int i = 0; i < N; i++) iniciarBarrido(i);
    Serial.println("Barrido de los 3 activado");
  } else if (cmd == "c") {
    mover(0, 90);
    mover(1, 90);
    mover(2, LAPIZ_ARRIBA);
  } else if (cmd.length() == 2 && cmd[0] == 's') {
    int i = cmd[1] - '1';
    if (i < 0 || i >= N) { invalido(); return; }
    iniciarBarrido(i);
    Serial.printf("Barrido del servo %d activado\n", i + 1);
  } else {
    int esp = cmd.indexOf(' ');
    if (esp < 0) { invalido(); return; }
    String quien = cmd.substring(0, esp);
    int valor;
    if (!aEntero(cmd.substring(esp + 1), valor)) { invalido(); return; }

    if (quien == "f") {
      if (valor > 7) { invalido(); return; }
      figura = valor;
      iniciarDibujo(CENTRO_S1, CENTRO_S2, figura);
      return;
    }
    if (quien == "exf") {
      if (valor > 15) { invalido(); return; }
      extraFinalMM = valor;
      Serial.printf("La ultima linea se pasa %d mm mas\n", extraFinalMM);
      return;
    }
    if (quien == "ex") {
      if (valor > 6) { invalido(); return; }
      extraMM = valor;
      Serial.printf("Cada linea se pasa %d mm en las esquinas\n", extraMM);
      return;
    }
    if (quien == "techo") {
      if (valor < 1 || valor > 4) { invalido(); return; }
      ladoTecho = valor;
      Serial.printf("Techo sobre el lado %d (1=AB 2=BC 3=CD 4=DA)\n", ladoTecho);
      return;
    }
    if (quien == "tm") {
      if (valor < 20 || valor > 500) { invalido(); return; }
      tMuestraUsuario = valor;
      Serial.printf("Tiempo entre muestras: %d ms\n", tMuestraUsuario);
      return;
    }
    if (quien == "pm") {
      if (valor < 3 || valor > 50) { invalido(); return; }
      pasoMin = valor / 10.0;
      Serial.printf("Paso por muestra: %.1f grados\n", pasoMin);
      return;
    }
    if (quien == "v") {
      if (valor < 1 || valor > 200) { invalido(); return; }
      tPaso = valor;
      Serial.printf("Velocidad: %lu ms por grado (%.1f grados/s)\n", tPaso, 1000.0 / tPaso);
      return;
    }

    if (valor > 180) { invalido(); return; }

    if (quien == "a") {
      for (int i = 0; i < N; i++) mover(i, valor);
    } else {
      int n;
      if (!aEntero(quien, n) || n < 1 || n > N) { invalido(); return; }
      mover(n - 1, valor);
    }
  }
}