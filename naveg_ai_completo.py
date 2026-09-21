import math

class NavMath:
    EARTH_RADIUS_NM = 3440.065  # Raio da Terra em Milhas Náuticas

    @staticmethod
    def haversine_distance_nm(lat1, lon1, lat2, lon2):
        """Calcula a distância em Milhas Náuticas (NM) entre dois pontos."""
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = (math.sin(dlat / 2) ** 2 + 
             math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2)
        c = 2 * math.asin(math.sqrt(max(0.0, min(1.0, a))))
        return NavMath.EARTH_RADIUS_NM * c

    @staticmethod
    def bearing_degrees(lat1, lon1, lat2, lon2):
        """Calcula o rumo inicial (bearing) em graus de (lat1, lon1) para (lat2, lon2)."""
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        y = math.sin(dlon) * math.cos(lat2)
        x = (math.cos(lat1) * math.sin(lat2) - 
             math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
        bearing = math.atan2(y, x)
        return (math.degrees(bearing) + 360) % 360

    @staticmethod
    def normalize_angle(angle):
        """Normaliza um ângulo para a faixa de -180 a +180 graus."""
        return (angle + 180) % 360 - 180

    @staticmethod
    def calculate_xtk_by_course(lat_curr, lon_curr, lat_orig, lon_orig, course_deg):
        """
        Calcula o Cross-Track Error (XTK) em Milhas Náuticas (NM) em relação a um curso dinâmico.
        Retorna:
          xtk_nm: Positivo se a aeronave estiver à DIREITA do curso, Negativo se estiver à ESQUERDA.
          dist_from_orig: Distância em NM da aeronave até o ponto de origem.
        """
        # Distância e bearing do ponto de origem até a posição atual
        dist_from_orig = NavMath.haversine_distance_nm(lat_orig, lon_orig, lat_curr, lon_curr)
        if dist_from_orig < 0.0001:
            return 0.0, 0.0

        bearing_from_orig = NavMath.bearing_degrees(lat_orig, lon_orig, lat_curr, lon_curr)

        # Diferença angular entre o rumo percorrido desde a origem e o curso desejado
        angle_diff = NavMath.normalize_angle(bearing_from_orig - course_deg)
        angle_diff_rad = math.radians(angle_diff)

        # Fórmula esférica para Cross-Track Distance
        sin_xt = math.sin(dist_from_orig / NavMath.EARTH_RADIUS_NM) * math.sin(angle_diff_rad)
        sin_xt = max(-1.0, min(1.0, sin_xt))
        xtk_nm = math.asin(sin_xt) * NavMath.EARTH_RADIUS_NM

        return xtk_nm, dist_from_orig

    @staticmethod
    def calculate_intercept_heading(course_deg, xtk_nm, sensitivity=30.0, max_intercept=45.0):
        """
        Calcula a proa sugerida pela IA para interceptar o curso.
        Parâmetros:
          course_deg: O curso original desejado.
          xtk_nm: O desvio lateral atual (positivo = direita, negativo = esquerda).
          sensitivity: Quantos graus de interceptação por NM de erro (padrão: 30° por NM).
          max_intercept: Ângulo máximo de corte (padrão: 45°).
        Retorna:
          target_heading: Proa magnética sugerida (0-359).
        """
        if abs(xtk_nm) <= 0.15:  # Considerado na rota (menos de 0.15 NM)
            return int(course_deg) % 360

        # Ângulo de corte proporcional ao desvio lateral
        intercept_angle = min(max_intercept, abs(xtk_nm) * sensitivity)
        # Se o desvio for mínimo, garante um corte mínimo de 10°
        intercept_angle = max(10.0, intercept_angle)

        if xtk_nm > 0:
            # Desvio para a direita: vire para a esquerda (subtraia o ângulo)
            target_heading = (course_deg - intercept_angle) % 360
        else:
            # Desvio para a esquerda: vire para a direita (some o ângulo)
            target_heading = (course_deg + intercept_angle) % 360

        return int(target_heading)

    @staticmethod
    def project_point(lat_deg, lon_deg, bearing_deg, dist_nm):
        """
        Projeta um ponto a 'dist_nm' NM de (lat_deg, lon_deg) no rumo 'bearing_deg'.
        Usa a fórmula do Problema Direto da geodésia esférica.
        Retorna (lat_nova, lon_nova) em graus decimais.
        """
        R = NavMath.EARTH_RADIUS_NM
        lat1 = math.radians(lat_deg)
        lon1 = math.radians(lon_deg)
        brg  = math.radians(bearing_deg)
        d_R  = dist_nm / R

        lat2 = math.asin(
            math.sin(lat1) * math.cos(d_R) +
            math.cos(lat1) * math.sin(d_R) * math.cos(brg)
        )
        lon2 = lon1 + math.atan2(
            math.sin(brg) * math.sin(d_R) * math.cos(lat1),
            math.cos(d_R) - math.sin(lat1) * math.sin(lat2)
        )
        return math.degrees(lat2), math.degrees(lon2)

    @staticmethod
    def generate_gs_waypoints(rwy_thr_lat, rwy_thr_lon, inbound_course_true,
                              inbound_course_mag, rwy_elev_ft,
                              num_points=10, max_dist_nm=10.0,
                              gs_deg=3.0, tch_ft=50):
        """
        Gera N waypoints para a rampa de Glide Slope sintético de 3° (ou gs_deg°).

        A rampa é ancorada nos 50 pés (tch_ft) acima da elevação da pista (rwy_elev_ft),
        exatamente sobre a cabeceira. Os waypoints são projetados para TRÁS da cabeceira
        (rumo recíproco do inbound_course_true) em intervalos de 1 NM até max_dist_nm.

        Parâmetros:
          rwy_thr_lat/lon  : Coordenadas da cabeceira da pista (posição atual da aeronave)
          inbound_course_true: Curso VERDADEIRO de aproximação (rumo para a pista)
          inbound_course_mag : Curso MAGNÉTICO de aproximação
          rwy_elev_ft      : Elevação da pista em pés
          num_points       : Número de waypoints gerados (padrão: 10)
          max_dist_nm      : Distância máxima da rampa em NM (padrão: 10 NM)
          gs_deg           : Ângulo da rampa em graus (padrão: 3.0°)
          tch_ft           : Threshold Crossing Height em pés (padrão: 50 ft)

        Retorna:
          Lista de dicts: {name, lat, lon, alt, course, course_true}
          O WP01 (dist=0) é a cabeceira. WP02 (1 NM), ..., WP11 (10 NM).
          Primeiro waypoint = cabeceira (índice 0, altitude = rwy_elev + tch)
          Waypoints seguintes afastam 1 NM para trás a cada passo.
        """
        # Conversão do ângulo de rampa: pés por NM
        # tan(3°) * 6076.12 ft/NM ≈ 318.4 ft/NM
        ft_per_nm = math.tan(math.radians(gs_deg)) * 6076.12

        # O rumo de AFASTAMENTO é o recíproco do inbound (de frente para trás da pista)
        outbound_true = (inbound_course_true + 180) % 360
        outbound_mag  = (inbound_course_mag  + 180) % 360

        step_nm = max_dist_nm / num_points  # 1 NM por passo com num_points=10

        waypoints = []
        for i in range(num_points + 1):  # 0 a 10 inclusive → 11 pontos
            dist_nm = i * step_nm
            alt_ft  = rwy_elev_ft + tch_ft + dist_nm * ft_per_nm

            lat, lon = NavMath.project_point(
                rwy_thr_lat, rwy_thr_lon, outbound_true, dist_nm
            )

            if i == 0:
                name = "RWY-THR"
            else:
                name = f"GS{i:02d}"

            waypoints.append({
                "name"        : name,
                "lat"         : lat,
                "lon"         : lon,
                "alt"         : round(alt_ft, 1),
                "course"      : outbound_mag,
                "course_true" : outbound_true,
                "desc"        : "GS_SYNTH",
            })

        # Os waypoints são da cabeceira para fora (0→10 NM).
        # Para navegar de fora para dentro, revertemos a lista e ajustamos o curso
        # para o inbound (rumo em direção à pista).
        waypoints.reverse()
        for wp in waypoints:
            wp["course"]       = inbound_course_mag
            wp["course_true"]  = inbound_course_true

        return waypoints


# Coordenadas e elevações das cabeceiras extraídas do AIP Brasil / AISWEB DECEA
# RWY16 threshold: S23°13'11" / W045°52'19"  Elev: 2032 ft
# RWY34 threshold: S23°14'12" / W045°51'12"  Elev: 2121 ft

SBSJ_RNP_16_SJ063_ROUTE = [
    {"name": "SJ063", "lat": -22.995180, "lon": -46.115061, "alt_ft": 8000, "desc": "IAF"},
    {"name": "SJ043", "lat": -23.030636, "lon": -46.076713, "alt_ft": 7000, "desc": "IF transição"},
    {"name": "GEMSO", "lat": -23.101513, "lon": -45.999958, "alt_ft": 5100, "desc": "IF"},
    {"name": "ANRAN", "lat": -23.160600, "lon": -45.935991, "alt_ft": 3680, "desc": "FAF"},
    {"name": "RWY16", "lat": -23.219722, "lon": -45.871944, "alt_ft": 2423, "desc": "MAPT (Pista 16)"}
]

SBSJ_RNP_16_LONES_ROUTE = [
    {"name": "LONES", "lat": -23.030516, "lon": -45.923333, "alt_ft": 7000, "desc": "IAF"},
    {"name": "GEMSO", "lat": -23.101513, "lon": -45.999958, "alt_ft": 5100, "desc": "IF"},
    {"name": "ANRAN", "lat": -23.160600, "lon": -45.935991, "alt_ft": 3680, "desc": "FAF"},
    {"name": "RWY16", "lat": -23.219722, "lon": -45.871944, "alt_ft": 2423, "desc": "MAPT (Pista 16)"}
]

SBSJ_RNP_34_DAGOV_ROUTE = [
    {"name": "DAGOV", "lat": -23.437333, "lon": -45.635305, "alt_ft": 7000, "desc": "IAF"},
    {"name": "SJ046", "lat": -23.354752, "lon": -45.725200, "alt_ft": 5300, "desc": "IF"},
    {"name": "SJ047", "lat": -23.295752, "lon": -45.789352, "alt_ft": 3770, "desc": "FAF"},
    {"name": "RWY34", "lat": -23.236667, "lon": -45.853333, "alt_ft": 2571, "desc": "MAPT (Pista 34)"}
]

SBSJ_RNP_34_SOTPI_ROUTE = [
    {"name": "SOTPI", "lat": -23.245444, "lon": -45.626411, "alt_ft": 7000, "desc": "IAF"},
    {"name": "SJ046", "lat": -23.354752, "lon": -45.725200, "alt_ft": 5300, "desc": "IF"},
    {"name": "SJ047", "lat": -23.295752, "lon": -45.789352, "alt_ft": 3770, "desc": "FAF"},
    {"name": "RWY34", "lat": -23.236667, "lon": -45.853333, "alt_ft": 2571, "desc": "MAPT (Pista 34)"}
]

# Dicionário central para facilitar o acesso pela interface
AVAILABLE_ROUTES = {
    "SBSJ 16 (SJ063)": {
        "waypoints": SBSJ_RNP_16_SJ063_ROUTE,
        "rwy_elev": 2032,    # Elevação oficial da cabeceira RWY16 (AIP Brasil / DECEA)
        "mahf_name": "LONES",
        "mahf_lat": -23.030516,
        "mahf_lon": -45.923333,
        "mahf_alt": 7000,
        "missed_turn": "L"
    },
    "SBSJ 16 (LONES)": {
        "waypoints": SBSJ_RNP_16_LONES_ROUTE,
        "rwy_elev": 2032,    # Elevação oficial da cabeceira RWY16 (AIP Brasil / DECEA)
        "mahf_name": "LONES",
        "mahf_lat": -23.030516,
        "mahf_lon": -45.923333,
        "mahf_alt": 7000,
        "missed_turn": "L"
    },
    "SBSJ 34 (DAGOV)": {
        "waypoints": SBSJ_RNP_34_DAGOV_ROUTE,
        "rwy_elev": 2121,    # Elevação oficial da cabeceira RWY34 (AIP Brasil / DECEA)
        "mahf_name": "SOTPI",
        "mahf_lat": -23.245444,
        "mahf_lon": -45.626411,
        "mahf_alt": 7000,
        "missed_turn": "R"
    },
    "SBSJ 34 (SOTPI)": {
        "waypoints": SBSJ_RNP_34_SOTPI_ROUTE,
        "rwy_elev": 2121,    # Elevação oficial da cabeceira RWY34 (AIP Brasil / DECEA)
        "mahf_name": "SOTPI",
        "mahf_lat": -23.245444,
        "mahf_lon": -45.626411,
        "mahf_alt": 7000,
        "missed_turn": "R"
    }
}


import math
import time
import random

# Tenta importar SimConnect (disponível no Windows com MSFS)
try:
    from SimConnect import AircraftRequests, SimConnect
    SIMCONNECT_AVAILABLE = True
except ImportError:
    SIMCONNECT_AVAILABLE = False

# Tenta importar bibliotecas da Raspberry Pi
try:
    import serial
    import pynmea2
    import board
    import busio
    import adafruit_bno055
    PI_SENSORS_AVAILABLE = True
except ImportError:
    PI_SENSORS_AVAILABLE = False


class TelemetrySource:
    def __init__(self):
        self.connected = False

    def connect(self):
        raise NotImplementedError

    def get_data(self):
        raise NotImplementedError

    def disconnect(self):
        pass


class SimConnectSource(TelemetrySource):
    def __init__(self):
        super().__init__()
        self.sm = None
        self.aq = None

    def connect(self):
        if not SIMCONNECT_AVAILABLE:
            print("SimConnect não está instalado neste ambiente Python.")
            self.connected = False
            return False
        try:
            print("Conectando ao Flight Simulator...")
            self.sm = SimConnect()
            self.aq = AircraftRequests(self.sm)
            self.connected = True
            print("Conectado ao Flight Simulator via SimConnect!")
            return True
        except Exception as e:
            print(f"Erro ao conectar ao Flight Simulator: {e}")
            self.connected = False
            return False

    def get_data(self):
        """
        Coleta os dados de telemetria em tempo real do Flight Simulator via SimConnect.
        Inverte o sinal do Pitch e do Bank para manter o horizonte artificial correto.
        Coleta proas magnéticas e verdadeiras para resolver problemas de desvios constantes.
        Todos os blocos do código foram totalmente comentados.
        """
        if not self.connected or not self.aq:
            return {"connected": False}

        try:
            # Latitude, Longitude (em graus decimais) e Altitude/Velocidade
            lat = self.aq.get("PLANE_LATITUDE")
            lon = self.aq.get("PLANE_LONGITUDE")
            alt = self.aq.get("INDICATED_ALTITUDE") or self.aq.get("PLANE_ALTITUDE") or 0.0
            
            # Coleta de Velocidade Indicada (IAS) em nós, para bater com o painel do avião
            speed = self.aq.get("AIRSPEED_INDICATED")
            if speed is None:
                # Caso a velocidade indicada falhe, faz fallback para Ground Speed
                gps_speed = self.aq.get("GPS_GROUND_SPEED")
                speed = (gps_speed * 1.94384) if gps_speed is not None else 0.0
            
            # Velocímetros reais perdem precisão ou sequer marcam abaixo de 30 nós (pressão do pitot muito baixa).
            # No simulador, o vento de proa com o avião estacionado pode gerar velocidades "fantasmas" (ex: 7 nós).
            # Esse filtro zera a leitura abaixo de 30 nós, reproduzindo o comportamento realista do PFD.
            if speed < 30.0:
                speed = 0.0
            
            # Pitch (arfar) em radianos: invertemos o sinal para o horizonte artificial no PFD
            pitch_rad = self.aq.get("PLANE_PITCH_DEGREES") or 0.0
            pitch = -math.degrees(pitch_rad)
            
            # Bank (rolar) em radianos: mantemos o sinal para alinhar o horizonte do PFD corretamente (correção de inversão de lado)
            bank_rad = self.aq.get("PLANE_BANK_DEGREES") or 0.0
            bank = math.degrees(bank_rad)
            
            # Proa Magnética (Heading) para exibição simples no painel
            heading_rad = self.aq.get("PLANE_HEADING_DEGREES_MAGNETIC") or 0.0
            heading = math.degrees(heading_rad) % 360
            
            # Rumo Magnético de Solo (Track) para exibição no painel
            track_rad = self.aq.get("GPS_GROUND_MAGNETIC_TRACK")
            if track_rad is not None:
                track = math.degrees(track_rad) % 360
            else:
                track = heading

            # Coleta de rumos Verdadeiros (True Heading e True Track) para cálculos geométricos
            # Isso elimina erros decorrentes da declinação magnética local (como os 21° W em SBSP)
            true_heading_rad = self.aq.get("PLANE_HEADING_DEGREES_TRUE") or 0.0
            true_heading = math.degrees(true_heading_rad) % 360
            
            true_track_rad = self.aq.get("GPS_GROUND_TRUE_TRACK")
            if true_track_rad is not None:
                true_track = math.degrees(true_track_rad) % 360
            else:
                true_track = true_heading

            # Declinação magnética local calculada dinamicamente: True Heading - Magnetic Heading
            mag_var = (true_heading - heading + 180) % 360 - 180

            # ── Estabilização do Track em solo ──────────────────────────────────
            # GPS_GROUND_TRACK oscila quando a velocidade de solo é próxima de zero.
            # Abaixo de 2 nós, usamos o heading magnético (bússola) como track estável.
            SPEED_THRESHOLD_KT = 2.0
            if speed < SPEED_THRESHOLD_KT:
                track = heading
                true_track = true_heading

            # Aguarda o carregamento completo das coordenadas do voo no simulador
            if lat is None or lon is None:
                return {"connected": False, "waiting_for_flight": True}

            return {
                "connected": True,
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "speed": speed,
                "heading": heading,
                "track": track,
                "true_heading": true_heading,
                "true_track": true_track,
                "mag_var": mag_var,
                "pitch": pitch,
                "bank": bank
            }
        except Exception as e:
            print(f"Erro ao ler telemetria do SimConnect: {e}")
            self.connected = False
            return {"connected": False}

    def disconnect(self):
        if self.sm:
            self.sm.exit()
            print("SimConnect desconectado.")
        self.connected = False


class PiSensorSource(TelemetrySource):
    def __init__(self, gps_port="/dev/ttyUSB0", gps_baud=9600):
        super().__init__()
        self.gps_port = gps_port
        self.gps_baud = gps_baud
        self.ser = None
        self.i2c = None
        self.imu = None
        
        # Últimos dados salvos para caso de falha de leitura rápida
        self.last_lat = -23.6277   # Default SBSP
        self.last_lon = -46.6546
        self.last_speed = 0.0
        self.last_track = 0.0
        self.last_alt = 2600.0

    def connect(self):
        if not PI_SENSORS_AVAILABLE:
            print("Bibliotecas de sensores da Raspberry Pi (serial, pynmea2, adafruit-circuitpython-bno055) não estão disponíveis.")
            self.connected = False
            return False
        
        # Conectar GPS via Porta Serial
        try:
            print(f"Conectando ao GPS Neo M8N na porta {self.gps_port}...")
            self.ser = serial.Serial(self.gps_port, self.gps_baud, timeout=0.5)
            print("GPS Inicializado!")
        except Exception as e:
            print(f"Aviso: Não foi possível abrir a porta serial do GPS: {e}")
            print("Tentando rodar com GPS simulado...")

        # Conectar IMU BNO055 via I2C
        try:
            print("Conectando ao sensor IMU BNO055 via I2C...")
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.imu = adafruit_bno055.BNO055_I2C(self.i2c)
            print("BNO055 Inicializado com sucesso!")
        except Exception as e:
            print(f"Erro ao inicializar sensor BNO055: {e}")
            self.connected = False
            return False

        self.connected = True
        return True

    def get_data(self):
        if not self.connected:
            return {"connected": False}

        # 1. Leitura do GPS (NMEA via Serial)
        if self.ser and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('ascii', errors='ignore')
                if line.startswith('$GPRMC') or line.startswith('$GNGGA'):
                    msg = pynmea2.parse(line)
                    # Verifica se o sinal do GPS está válido
                    if hasattr(msg, 'is_valid') and msg.is_valid:
                        self.last_lat = msg.latitude
                        self.last_longitude = msg.longitude
                        if hasattr(msg, 'spd_over_grnd') and msg.spd_over_grnd is not None:
                            self.last_speed = float(msg.spd_over_grnd) # O NMEA ($GPRMC) já reporta a Ground Speed em nós (knots)
                        if hasattr(msg, 'true_course') and msg.true_course is not None:
                            self.last_track = msg.true_course
                        if hasattr(msg, 'altitude') and msg.altitude is not None:
                            self.last_alt = msg.altitude * 3.28084  # metros para pés
            except Exception as e:
                # Silencia erros de parseamento rápido da serial
                pass

        # 2. Leitura da IMU BNO055 (Orientação Absoluta)
        pitch = 0.0
        bank = 0.0
        heading = self.last_track

        if self.imu:
            try:
                # euler retorna: (heading/yaw, roll/bank, pitch)
                euler = self.imu.euler
                if euler and euler[0] is not None:
                    heading = euler[0]
                    # No BNO055, dependendo da orientação física,
                    # roll e pitch podem precisar de mapeamento ou inversão de sinal
                    # euler[1] = roll, euler[2] = pitch
                    bank = euler[1] if euler[1] is not None else 0.0
                    pitch = euler[2] if euler[2] is not None else 0.0
            except Exception as e:
                # Silencia erros de leitura I2C rápida
                pass

        # O GPS Neo M8N fornece coordenadas geográficas e rumos baseados no norte verdadeiro (True).
        # A IMU BNO055 fornece orientação baseada no norte magnético (Magnetic).
        # Em modo Pi, tratamos true_track como o track do GPS e definimos variação nula caso não especificado.
        true_heading = heading
        true_track = self.last_track
        mag_var = 0.0

        return {
            "connected": True,
            "lat": self.last_lat,
            "lon": self.last_lon,
            "alt": self.last_alt,
            "speed": self.last_speed,
            "heading": heading,
            "track": self.last_track,
            "true_heading": true_heading,
            "true_track": true_track,
            "mag_var": mag_var,
            "pitch": pitch,
            "bank": bank
        }

    def disconnect(self):
        if self.ser:
            self.ser.close()
        self.connected = False


class MockSource(TelemetrySource):
    """Fonte de dados simulada para desenvolvimento e testes locais."""
    def __init__(self):
        super().__init__()
        self.lat = -23.5650
        self.lon = -46.8283
        self.alt = 5500.0
        self.speed = 120.0
        self.heading = 78.0
        self.track = 78.0
        self.pitch = 0.0
        self.bank = 0.0
        self.last_time = time.time()
        
        # Variáveis para simular desvios controlados
        self.drift_mode = "none" # "left", "right"
        self.drift_rate = 0.00005 # Taxa de desvio em graus de latitude/longitude por segundo

    def connect(self):
        self.connected = True
        self.last_time = time.time()
        print("Mock de telemetria conectado!")
        return True

    def get_data(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # Simula pequenos ruídos e variações normais de voo
        self.pitch += random.uniform(-0.1, 0.1)
        self.pitch = max(-15.0, min(15.0, self.pitch))
        
        self.bank += random.uniform(-0.2, 0.2)
        self.bank = max(-30.0, min(30.0, self.bank))

        # Modifica heading lentamente dependendo do bank
        self.heading = (self.heading + self.bank * dt * 0.5) % 360
        self.track = self.heading

        # Simula o avião se movendo na direção da proa (em graus decimais)
        heading_rad = math.radians(self.heading)
        # Velocidade em graus por segundo (aproximado)
        speed_deg_sec = (self.speed / 3600.0) / 60.0 # Aproximação grosseira para movimento
        
        self.lat += math.cos(heading_rad) * speed_deg_sec * dt
        self.lon += math.sin(heading_rad) * speed_deg_sec * dt

        # Adiciona desvios artificiais (derivas laterais)
        if self.drift_mode == "left":
            # Empurra o avião para a esquerda perpendicularmente à trajetória (somando 90 graus)
            drift_rad = math.radians(self.heading - 90)
            self.lat += math.cos(drift_rad) * self.drift_rate * dt
            self.lon += math.sin(drift_rad) * self.drift_rate * dt
        elif self.drift_mode == "right":
            # Empurra o avião para a direita (somando 90 graus)
            drift_rad = math.radians(self.heading + 90)
            self.lat += math.cos(drift_rad) * self.drift_rate * dt
            self.lon += math.sin(drift_rad) * self.drift_rate * dt

        # No simulador local (Mock), assumimos que os rumos magnéticos e verdadeiros coincidem (mag_var = 0.0)
        true_heading = self.heading
        true_track = self.track
        mag_var = 0.0

        return {
            "connected": True,
            "lat": self.lat,
            "lon": self.lon,
            "alt": self.alt,
            "speed": self.speed,
            "heading": self.heading,
            "track": self.track,
            "true_heading": true_heading,
            "true_track": true_track,
            "mag_var": mag_var,
            "pitch": self.pitch,
            "bank": self.bank
        }


import pygame
import math

# Cores do Sistema
WHITE = (240, 240, 240)
BLACK = (10, 10, 10)
GRAY = (60, 60, 65)
LIGHT_GRAY = (150, 150, 150)
MAGENTA = (255, 0, 255)
SKY = (65, 135, 210)
GROUND = (130, 80, 40)
GREEN = (40, 210, 90)
RED = (225, 55, 55)
YELLOW = (255, 200, 0)
BLUE = (50, 130, 220)
PANEL_BG = (22, 22, 26)
BOX_BG = (32, 32, 36)
AI_PURPLE = (150, 110, 255)

# Resoluções e proporções
BASE_WINDOW_SIZE = (1800, 900)
WINDOW_SIZE = (1350, 675)

# Configurações do Horizonte Artificial (Esquerda)
CENTER = (280, 380)
RADIUS = 185
DIAMETER = RADIUS * 2
PITCH_SCALE = 4


class PFDGui:
    def __init__(self, base_size=(1800, 900)):
        self.base_size = base_size
        
        # Inicializa as fontes do cockpit
        self.f_title = pygame.font.SysFont("Segoe UI", 28, bold=True)
        self.f_label = pygame.font.SysFont("Segoe UI", 16, bold=True)
        self.f_data = pygame.font.SysFont("Consolas", 42, bold=True)
        self.f_data_compact = pygame.font.SysFont("Consolas", 28, bold=True)  # Para valores longos (5+ dígitos)
        self.f_msg = pygame.font.SysFont("Segoe UI", 22, italic=True)
        self.f_btn = pygame.font.SysFont("Segoe UI", 22, bold=True)
        self.f_small = pygame.font.SysFont("Consolas", 14, bold=True)
        self.f_field_label = pygame.font.SysFont("Segoe UI", 18, bold=True)
        
        # Linha 1 - Botão INICIAR/PARAR monitor
        self.btn_monitor = pygame.Rect(600, 100, 350, 54)
        
        # Linha 2 - Lado Esquerdo: Entrada do Curso Manual (redimensionado para caber GS)
        self.course_input_rect = pygame.Rect(600, 190, 130, 46)
        self.btn_set_course = pygame.Rect(740, 190, 75, 46)

        # Linha 2 - Centro-Esquerdo: Tolerância XTK customizada (redimensionado)
        self.tolerance_input_rect = pygame.Rect(825, 190, 110, 46)
        self.btn_set_tolerance = pygame.Rect(945, 190, 65, 46)

        # Linha 2 - Lado Direito: Controles do Glide Slope Sintético
        # Campo para digitar a altitude da pista (elevação do aeródromo)
        self.gs_rwy_alt_input_rect = pygame.Rect(1020, 190, 148, 46)
        # Botão INICIAR 50FT ajustado para não invadir a label superior
        self.btn_start_gs = pygame.Rect(1178, 180, 130, 56)

        # Linha 3 - Botões de Waypoint: GRAVAR | NAVEGAR/REINICIAR | ZERAR (3 botões iguais)
        self.btn_record_wp  = pygame.Rect(600,  250, 230, 42)
        self.btn_return_wp  = pygame.Rect(840,  250, 200, 42)
        self.btn_clear_wps  = pygame.Rect(1050, 250, 210, 42)
        
        # Limite padrão para alerta de desvio lateral (XTK)
        # Inicializado em 0.3 NM por padrão. Será atualizado dinamicamente pelo nav_state.
        self.xtk_tolerance = 0.3

        # Controles VNAV (Painel Direito Extra)
        self.vnav_cal_input_rect = pygame.Rect(1380, 200, 200, 46)
        self.btn_toggle_vnav_cal = pygame.Rect(1600, 200, 110, 46)

        self.vnav_alt_input_rect = pygame.Rect(1380, 290, 200, 46)
        self.btn_set_vnav_alt = pygame.Rect(1600, 290, 110, 46)
        self.vnav_tol_input_rect = pygame.Rect(1380, 380, 200, 46)
        self.btn_set_vnav_tol = pygame.Rect(1600, 380, 110, 46)

        # Controles Monitor de Procedimento (Painel Direito Extra Inferior)
        self.btn_cycle_chart = pygame.Rect(1380, 670, 360, 50)
        self.btn_load_chart = pygame.Rect(1380, 740, 360, 60)


    def draw_text(self, surface, text, pos, font, color=WHITE, align="left"):
        """Método utilitário para desenhar textos alinhados."""
        img = font.render(str(text), True, color)
        rect = img.get_rect()
        if align == "center":
            rect.center = pos
        elif align == "right":
            rect.topright = pos
        else:
            rect.topleft = pos
        surface.blit(img, rect)

    def draw_data_box(self, surface, rect, title, value, color=WHITE):
        """Desenha uma caixinha de dados de telemetria (HDG, TRK, SPD, ALT)."""
        pygame.draw.rect(surface, PANEL_BG, rect, border_radius=10)
        pygame.draw.rect(surface, GRAY, rect, 2, border_radius=10)
        self.draw_text(surface, title, (rect.centerx, rect.y + 12), self.f_label, WHITE, align="center")
        # Usa fonte compacta automaticamente se o valor tiver 5 ou mais caracteres
        font = self.f_data_compact if len(str(value)) >= 5 else self.f_data
        self.draw_text(surface, value, (rect.centerx, rect.y + 44), font, color, align="center")

    def draw_horizon(self, surface, pitch, bank):
        """Desenha o Horizonte Artificial na tela."""
        world_size = DIAMETER * 3
        world = pygame.Surface((world_size, world_size), pygame.SRCALPHA)
        world_center = world_size // 2
        
        # O pitch já vem corrigido do data_sources.py
        horizon_y = world_center + int(pitch * PITCH_SCALE)

        # Desenha o céu e a terra
        pygame.draw.rect(world, SKY, (0, 0, world_size, horizon_y))
        pygame.draw.rect(world, GROUND, (0, horizon_y, world_size, world_size - horizon_y))
        pygame.draw.line(world, WHITE, (0, horizon_y), (world_size, horizon_y), 3)

        # Escalas de pitch de 5 em 5 graus
        for mark in range(-30, 31, 5):
            if mark == 0:
                continue

            line_y = world_center + int((pitch - mark) * PITCH_SCALE)
            if not 0 <= line_y <= world_size:
                continue

            line_half = 66 if mark % 10 else 100
            pygame.draw.line(
                world,
                WHITE,
                (world_center - line_half, line_y),
                (world_center + line_half, line_y),
                2,
            )

            if mark % 10 == 0:
                label = str(abs(mark))
                self.draw_text(world, label, (world_center - line_half - 18, line_y - 10), self.f_label, align="right")
                self.draw_text(world, label, (world_center + line_half + 18, line_y - 10), self.f_label)

        # Rotaciona a superfície com base na inclinação das asas (bank)
        rotated = pygame.transform.rotozoom(world, -bank, 1.0)

        # Aplica a máscara circular do instrumento
        instrument = pygame.Surface((DIAMETER, DIAMETER), pygame.SRCALPHA)
        rotated_rect = rotated.get_rect(center=(RADIUS, RADIUS))
        instrument.blit(rotated, rotated_rect)

        mask = pygame.Surface((DIAMETER, DIAMETER), pygame.SRCALPHA)
        pygame.draw.circle(mask, (255, 255, 255, 255), (RADIUS, RADIUS), RADIUS)
        instrument.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)

        # Blit na tela principal
        surface.blit(instrument, (CENTER[0] - RADIUS, CENTER[1] - RADIUS))
        
        # Aro de bank
        pygame.draw.circle(surface, GRAY, CENTER, RADIUS, 3)
        self.draw_bank_arc(surface, bank)

    def draw_bank_arc(self, surface, bank):
        """Desenha a escala circular de inclinação (Bank)."""
        for angle in range(-60, 61, 15):
            rad = math.radians(angle - 90)
            x1 = CENTER[0] + int((RADIUS - 10) * math.cos(rad))
            y1 = CENTER[1] + int((RADIUS - 10) * math.sin(rad))
            x2 = CENTER[0] + int((RADIUS - 28) * math.cos(rad))
            y2 = CENTER[1] + int((RADIUS - 28) * math.sin(rad))
            pygame.draw.line(surface, WHITE, (x1, y1), (x2, y2), 2)

        bank_angle = math.radians(bank - 90)
        bx = CENTER[0] + int((RADIUS - 38) * math.cos(bank_angle))
        by = CENTER[1] + int((RADIUS - 38) * math.sin(bank_angle))
        pygame.draw.circle(surface, MAGENTA, (bx, by), 6)

    def draw_aircraft_reference(self, surface):
        """Desenha o símbolo do avião no centro do horizonte."""
        pygame.draw.line(surface, WHITE, (CENTER[0] - 30, CENTER[1]), (CENTER[0] + 30, CENTER[1]), 4)
        pygame.draw.line(surface, WHITE, (CENTER[0], CENTER[1] - 20), (CENTER[0], CENTER[1] + 20), 4)

    def draw_flight_director(self, surface, nav_state, current_alt):
        """
        Desenha as barras transversais do Flight Director (FD) em magenta sobre o Horizonte Artificial.
        Esta função implementa o sistema de comandos de voo (Flight Director System), que traduz 
        os erros de navegação em indicações visuais de atitude recomendada (pitch/roll).
        
        - Barra Vertical (LNAV): Move-se lateralmente indicando erro de rastreio (Cross-Track Error).
        - Barra Horizontal (VNAV): Move-se verticalmente indicando erro de Altitude.
        As barras são desenhadas de forma ortogonal, formando uma cruz no centro.
        A referência fixa do avião (mini-avião branco) é desenhada posteriormente, por cima do FD.
        """
        # Limites máximos físicos do instrumento na tela (em pixels) para o deslocamento das barras.
        # Esses valores impedem que as barras saiam do círculo do horizonte artificial.
        max_x_offset = 120
        max_y_offset = 120
        
        is_active = nav_state.get("active", False)
        v_phase = nav_state.get("vectoring_phase", 0)
        is_ma_climbing = nav_state.get("missed_approach_climbing", False)
        
        # Oculta o FD se o sistema não estiver engajado, nas fases 1 a 5, ou em subida de arremetida
        if not is_active or (1 <= v_phase <= 5) or is_ma_climbing:
            return

        # Modo GS Sintético: oculta o FD enquanto dist > 10 NM da cabeceira
        # A partir das 10 NM o FD acende e guia a descida pela rampa de 3°
        if nav_state.get("gs_synthetic_active") and not nav_state.get("active_procedure"):
            dist_thr = nav_state.get("gs_dist_to_threshold", 999.0)
            if dist_thr > 10.0:
                return
            
        # --- LNAV (Barra Vertical) ---
        # A barra vertical atua como um comando de Roll (Rolamento).
        xtk_nm = nav_state.get("xtk", 0.0)
        
        # O deslocamento da barra é calculado proporcionalmente ao erro (XTK) e à sensibilidade desejada (xtk_tolerance).
        # Se xtk > 0 (o avião está fisicamente à direita da rota), a barra deve deslocar para a esquerda (comando de curvar à esquerda).
        # Para refletir esse movimento natural ("Voe para a Agulha"), invertemos o sinal do XTK (-xtk_nm).
        ratio_x = -xtk_nm / self.xtk_tolerance
        
        # A saturação (clipping) limita a barra às bordas do instrumento se o erro exceder a tolerância.
        ratio_x = max(-1.0, min(1.0, ratio_x))
        x_offset = int(ratio_x * max_x_offset)
        
        # Renderiza a Barra Vertical Magenta
        fd_x = CENTER[0] + x_offset
        pygame.draw.line(surface, MAGENTA, (fd_x, CENTER[1] - 100), (fd_x, CENTER[1] + 100), 3)

        # --- VNAV (Barra Horizontal) ---
        # A barra horizontal atua como um comando de Pitch (Arfagem).
        # Tenta usar a altitude do Glidepath (se existir), senão usa a estática
        vnav_gs = nav_state.get("vnav_gs_alt")
        target_alt = vnav_gs if vnav_gs is not None else nav_state.get("vnav_target_alt")
        if target_alt is not None:
            vnav_tolerance = nav_state.get("vnav_tolerance", 200)
            alt_diff = current_alt - target_alt
            
            # O cálculo do desvio de altitude obedece a lógica proporcional baseada na tolerância escolhida (ex: 200 ft).
            # Se alt_diff > 0 (o avião está mais alto que o alvo), a barra deve deslocar para cima na tela.
            # No pygame, coordenadas Y aumentam para baixo. Um deslocamento positivo do offsetY abaixa a barra na tela, 
            # comandando o piloto a "empurrar o manche" (Pitch Down), buscando a agulha ("Fly to Needle").
            # Se alt_diff < 0 (o avião está abaixo), o offsetY fica negativo, subindo a barra e comandando Pitch Up.
            ratio_y = alt_diff / vnav_tolerance
            ratio_y = max(-1.0, min(1.0, ratio_y))
            y_offset = int(ratio_y * max_y_offset)
            
            # Renderiza a Barra Horizontal Magenta
            fd_y = CENTER[1] + y_offset
            pygame.draw.line(surface, MAGENTA, (CENTER[0] - 100, fd_y), (CENTER[0] + 100, fd_y), 3)
            
            # Se a rampa dinâmica (Glidepath) estiver ativa, exibe "GS" em verde
            if nav_state.get("vnav_gs_alt") is not None:
                self.draw_text(surface, "GS", (CENTER[0] + 110, fd_y - 10), self.f_label, GREEN)

    def draw_waypoint_scale(self, surface, waypoints, active_index, nav_mode):
        """
        Desenha o trilho vertical de waypoints no painel esquerdo.
        Posicionado abaixo dos instrumentos (ALT/SPD) para não obstruir o horizonte.
        Sempre exibe os 10 slots possíveis:
          - Slot vazio: círculo pequeno cinza (disponível)
          - Slot preenchido: círculo sólido colorido com nome do WP
          - WP ativo (modo rota): pisca entre amarelo e magenta, maior
          - WP concluído: verde
        Seta na base indica a direção do voo.
        """
        MAX_WPS = 10
        # Posicionado no canto direito do painel esquerdo, abaixo dos instrumentos
        scale_x     = 490      # Centro X - recuado para dar espaço à numeração
        scale_top   = 440      # Abaixo da caixa ALT/SPD (y=338+86=424)
        scale_bottom= 870      # Até o fundo da tela
        scale_height = scale_bottom - scale_top

        # ── Fundo semitransparente do trilho (estreito) ───────────────────────
        bg_rect = pygame.Rect(scale_x - 28, scale_top - 6, 90, scale_height + 12)
        bg_surf = pygame.Surface((bg_rect.width, bg_rect.height), pygame.SRCALPHA)
        bg_surf.fill((10, 12, 22, 180))
        surface.blit(bg_surf, bg_rect)
        pygame.draw.rect(surface, (60, 60, 80), bg_rect, 1, border_radius=6)

        # ── Título ───────────────────────────────────────────────────
        self.draw_text(surface, "ROTA", (scale_x, scale_top + 2), self.f_small, LIGHT_GRAY, align="center")

        # ── Linha central vertical do trilho ───────────────────────────
        pygame.draw.line(surface, (50, 50, 70),
                         (scale_x, scale_top + 16), (scale_x, scale_bottom - 16), 2)

        # ── Seta de direção na base ───────────────────────────────
        ay = scale_bottom - 10
        pygame.draw.polygon(surface, (80, 80, 110), [
            (scale_x,     ay + 8),
            (scale_x - 6, ay - 2),
            (scale_x + 6, ay - 2)
        ])

        # ── Espaçamento entre os 10 slots ────────────────────────────
        slot_spacing = (scale_height - 34) / (MAX_WPS - 1)

        for i in range(MAX_WPS):
            y = int(scale_top + 18 + i * slot_spacing)

            if i < len(waypoints):
                wp    = waypoints[i]
                label = wp.get("name", f"WYP{i+1:02d}")

                if nav_mode == "route":
                    if i < active_index:
                        color, radius, border = GREEN, 7, 0
                    elif i == active_index:
                        color  = YELLOW if (pygame.time.get_ticks() // 400) % 2 == 0 else MAGENTA
                        radius, border = 10, 2
                    else:
                        color, radius, border = LIGHT_GRAY, 6, 1
                else:
                    color, radius, border = (80, 160, 255), 7, 1

                pygame.draw.circle(surface, color, (scale_x, y), radius)
                if border:
                    pygame.draw.circle(surface, WHITE, (scale_x, y), radius, border)

                # Linha de conexão entre slots preenchidos consecutivos
                if i > 0 and i - 1 < len(waypoints):
                    prev_y = int(scale_top + 18 + (i - 1) * slot_spacing)
                    conn   = GREEN if (nav_mode == "route" and i <= active_index) else (80, 160, 255)
                    pygame.draw.line(surface, conn,
                                     (scale_x, prev_y + radius),
                                     (scale_x, y - radius), 2)

                self.draw_text(surface, label, (scale_x + 13, y - 7), self.f_small, color)

            else:
                # Slot vazio
                pygame.draw.circle(surface, (45, 45, 60), (scale_x, y), 4)
                pygame.draw.circle(surface, (65, 65, 80), (scale_x, y), 4, 1)
                self.draw_text(surface, f"{i+1:02d}", (scale_x + 11, y - 7), self.f_small, (55, 55, 70))





    def draw_left_cdi(self, surface, xtk_nm, is_active):
        """
        Desenha o CDI no painel ESQUERDO, abaixo do horizonte artificial.
        Versão compacta (scale_width=320) centrada em X=280, Y=635.
        """
        center_x = 280
        center_y = 635
        scale_width = 320
        scale_half_w = scale_width // 2
        scale_height = 100
        scale_half_h = scale_height // 2
        max_scale_nm = 0.10

        # Fundo e borda
        pygame.draw.rect(surface, BOX_BG,
                         (center_x - scale_half_w - 16, center_y - scale_half_h - 8,
                          scale_width + 32, scale_height + 16), border_radius=10)
        pygame.draw.rect(surface, GRAY,
                         (center_x - scale_half_w - 16, center_y - scale_half_h - 8,
                          scale_width + 32, scale_height + 16), 1, border_radius=10)

        # Linha central de trajetória
        pygame.draw.line(surface, GRAY,
                         (center_x - scale_half_w, center_y),
                         (center_x + scale_half_w, center_y), 3)

        # Animação de setas quando ativo
        if is_active:
            t = pygame.time.get_ticks()
            spacing = scale_width // 5
            offset_anim = (t // 8) % spacing
            for i in range(6):
                x = center_x - scale_half_w + (i * spacing + offset_anim) % scale_width
                if center_x - scale_half_w <= x <= center_x + scale_half_w:
                    arrow_color = GREEN if (t // 500) % 2 == 0 else BLUE
                    pygame.draw.line(surface, arrow_color, (x, center_y - 5), (x + 5, center_y), 2)
                    pygame.draw.line(surface, arrow_color, (x + 5, center_y), (x, center_y + 5), 2)

        # Pontos de escala
        for i in range(1, 5):
            offset_y = int((i / 4) * (scale_half_h - 8))
            pygame.draw.circle(surface, LIGHT_GRAY, (center_x, center_y - offset_y), 3)
            pygame.draw.circle(surface, LIGHT_GRAY, (center_x, center_y + offset_y), 3)

        # Cursor (losango)
        needle_color = GREEN if is_active else LIGHT_GRAY
        if is_active:
            if abs(xtk_nm) > self.xtk_tolerance:
                if (pygame.time.get_ticks() // 250) % 2 == 0:
                    needle_color = RED

            ratio = xtk_nm / max_scale_nm
            ratio = max(-1.0, min(1.0, ratio))
            cursor_y = center_y + int(ratio * (scale_half_h - 8))

            points = [
                (center_x, cursor_y - 13),
                (center_x + 7, cursor_y),
                (center_x, cursor_y + 13),
                (center_x - 7, cursor_y)
            ]
            pygame.draw.polygon(surface, needle_color, points)
            pygame.draw.polygon(surface, WHITE, points, 2)

            # Label CDI acima do instrumento
            self.draw_text(surface, "CDI", (center_x, center_y - scale_half_h - 22),
                           self.f_label, LIGHT_GRAY, align="center")
        else:
            pygame.draw.circle(surface, GRAY, (center_x, center_y), 6, 2)

    def draw_navigation_panel(self, surface, data, nav_state, mouse_pos, active_input, course_text, tolerance_text, gs_rwy_alt_text=""):
        """
        Desenha o painel de navegação lateral (lado direito da tela).

        Exibe os botões de controle, campos de entrada e informações da IA.
        Inclui botões de GRAVAR WAYPOINT e RETORNAR WAYPOINT (Linha 3).
        Todos os blocos do código foram totalmente comentados.
        """
        panel_rect = pygame.Rect(560, 24, 756, 852)
        pygame.draw.rect(surface, PANEL_BG, panel_rect, border_radius=14)
        pygame.draw.rect(surface, GRAY, panel_rect, 2, border_radius=14)

        # Título do Painel
        self.draw_text(surface, "SISTEMA DE NAVEGAÇÃO LATERAL", (panel_rect.centerx, 50), self.f_title, AI_PURPLE, align="center")

        is_active = nav_state["active"]
        xtk = nav_state["xtk"]
        course = nav_state["course"]
        target_hdg = nav_state["target_heading"]
        waypoints = nav_state.get("recorded_waypoints", [])
        nav_mode = nav_state.get("navigation_mode", "manual")
        active_wp_idx = nav_state.get("active_waypoint_index", 0)
        
        # Sincroniza a tolerância interna do PFD com o valor configurado no nav_state
        self.xtk_tolerance = nav_state.get("xtk_tolerance", 0.3)

        # Indicador de Status Visual (ATIVO / STANDBY)
        status_color = GREEN if is_active else GRAY
        status_text = "ATIVO" if is_active else "STANDBY"
        pygame.draw.circle(surface, status_color, (610, 105), 10)
        self.draw_text(surface, f"STATUS: {status_text}", (630, 95), self.f_label, status_color)

        # ── LINHA 1: Botão Monitor ──────────────────────────
        system_on = nav_state.get("system_on", False)
        btn_monitor_color = RED if system_on else GREEN
        if self.btn_monitor.collidepoint(mouse_pos):
            btn_monitor_color = (max(0, btn_monitor_color[0]-40), max(0, btn_monitor_color[1]-40), max(0, btn_monitor_color[2]-40))
        pygame.draw.rect(surface, btn_monitor_color, self.btn_monitor, border_radius=10)
        pygame.draw.rect(surface, WHITE, self.btn_monitor, 2, border_radius=10)
        btn_label = "PARAR MONITOR" if system_on else "INICIAR MONITOR"
        self.draw_text(surface, btn_label, self.btn_monitor.center, self.f_btn, BLACK if not system_on else WHITE, align="center")

        # ── LINHA 2: Curso desejado (compacto) ───────────────────────────────
        is_course_active = (active_input == "course")
        course_border = BLUE if is_course_active else GRAY
        pygame.draw.rect(surface, BOX_BG, self.course_input_rect, border_radius=10)
        pygame.draw.rect(surface, course_border, self.course_input_rect, 2, border_radius=10)
        self.draw_text(surface, "Curso (0-359):", (self.course_input_rect.x, self.course_input_rect.y - 25), self.f_field_label, WHITE)
        
        display_course = course_text + ("|" if is_course_active and (pygame.time.get_ticks() // 400) % 2 == 0 else "")
        if not course_text and not is_course_active:
            self.draw_text(surface, "ex: 170", (self.course_input_rect.x + 8, self.course_input_rect.y + 12), self.f_small, GRAY)
        else:
            self.draw_text(surface, display_course, (self.course_input_rect.x + 8, self.course_input_rect.y + 10), self.f_data_compact, WHITE)

        btn_course_color = (100, 100, 105) if self.btn_set_course.collidepoint(mouse_pos) else GRAY
        pygame.draw.rect(surface, btn_course_color, self.btn_set_course, border_radius=10)
        pygame.draw.rect(surface, WHITE, self.btn_set_course, 1, border_radius=10)
        self.draw_text(surface, "SET", self.btn_set_course.center, self.f_small, WHITE, align="center")

        # ── LINHA 2: Tolerância XTK (compacta) ──────────────────────────────
        is_tolerance_active = (active_input == "tolerance")
        tolerance_border = BLUE if is_tolerance_active else GRAY
        pygame.draw.rect(surface, BOX_BG, self.tolerance_input_rect, border_radius=10)
        pygame.draw.rect(surface, tolerance_border, self.tolerance_input_rect, 2, border_radius=10)
        self.draw_text(surface, "XTK(NM):", (self.tolerance_input_rect.x, self.tolerance_input_rect.y - 25), self.f_field_label, WHITE)
        
        display_tolerance = tolerance_text + ("|" if is_tolerance_active and (pygame.time.get_ticks() // 400) % 2 == 0 else "")
        if not tolerance_text and not is_tolerance_active:
            self.draw_text(surface, "0.3", (self.tolerance_input_rect.x + 8, self.tolerance_input_rect.y + 12), self.f_small, GRAY)
        else:
            self.draw_text(surface, display_tolerance, (self.tolerance_input_rect.x + 8, self.tolerance_input_rect.y + 10), self.f_data_compact, WHITE)

        btn_tolerance_color = (100, 100, 105) if self.btn_set_tolerance.collidepoint(mouse_pos) else GRAY
        pygame.draw.rect(surface, btn_tolerance_color, self.btn_set_tolerance, border_radius=10)
        pygame.draw.rect(surface, WHITE, self.btn_set_tolerance, 1, border_radius=10)
        self.draw_text(surface, "SET", self.btn_set_tolerance.center, self.f_small, WHITE, align="center")

        # ── LINHA 2: Glide Slope Sintético (campo ALTITUDE RWY + botão INICIAR 50FT) ──
        gs_active = nav_state.get("gs_synthetic_active", False)
        is_gs_input_active = (active_input == "gs_rwy_alt")

        # Campo ALTITUDE RWY
        gs_border = BLUE if is_gs_input_active else (GREEN if gs_active else GRAY)
        pygame.draw.rect(surface, BOX_BG, self.gs_rwy_alt_input_rect, border_radius=10)
        pygame.draw.rect(surface, gs_border, self.gs_rwy_alt_input_rect, 2, border_radius=10)
        self.draw_text(surface, "ALT RWY(FT):", (self.gs_rwy_alt_input_rect.x, self.gs_rwy_alt_input_rect.y - 25), self.f_field_label, WHITE)
        display_gs_alt = gs_rwy_alt_text + ("|" if is_gs_input_active and (pygame.time.get_ticks() // 400) % 2 == 0 else "")
        if not gs_rwy_alt_text and not is_gs_input_active:
            stored = nav_state.get("gs_rwy_elev")
            hint = f"{int(stored)}" if stored is not None else "ex: 2100"
            self.draw_text(surface, hint, (self.gs_rwy_alt_input_rect.x + 8, self.gs_rwy_alt_input_rect.y + 12), self.f_small, GRAY)
        else:
            self.draw_text(surface, display_gs_alt, (self.gs_rwy_alt_input_rect.x + 8, self.gs_rwy_alt_input_rect.y + 10), self.f_data_compact, WHITE)

        # Botão INICIAR 50FT (2 linhas)
        gs_btn_base = (0, 180, 60) if gs_active else (80, 40, 120)
        if self.btn_start_gs.collidepoint(mouse_pos):
            gs_btn_base = (max(0, gs_btn_base[0]-30), max(0, gs_btn_base[1]-30), max(0, gs_btn_base[2]-30))
        pygame.draw.rect(surface, gs_btn_base, self.btn_start_gs, border_radius=10)
        pygame.draw.rect(surface, WHITE, self.btn_start_gs, 2, border_radius=10)
        btn_gs_line1 = "ATIVO GS" if gs_active else "INICIAR"
        btn_gs_line2 = "50FT"
        # Texto preto quando ativo (verde claro), branco quando inativo (roxo escuro)
        gs_txt_color = BLACK if gs_active else WHITE
        self.draw_text(surface, btn_gs_line1, (self.btn_start_gs.centerx, self.btn_start_gs.y + 10), self.f_small, gs_txt_color, align="center")
        self.draw_text(surface, btn_gs_line2, (self.btn_start_gs.centerx, self.btn_start_gs.y + 28), self.f_btn, gs_txt_color, align="center")

        # ── LINHA 3: Botões de Waypoint ───────────────────────────────────────
        wp_count = len(waypoints)
        max_wps = 10

        # Botão GRAVAR WAYPOINT
        display_count = min(wp_count, max_wps)
        record_label = f"GRAVAR WYP ({display_count}/{max_wps})"
        rec_base = BLUE if wp_count < max_wps else GRAY
        rec_color = (max(0, rec_base[0]-40), max(0, rec_base[1]-40), max(0, rec_base[2]-40)) \
                    if self.btn_record_wp.collidepoint(mouse_pos) else rec_base
        pygame.draw.rect(surface, rec_color, self.btn_record_wp, border_radius=10)
        pygame.draw.rect(surface, WHITE, self.btn_record_wp, 1, border_radius=10)
        self.draw_text(surface, record_label, self.btn_record_wp.center, self.f_btn, WHITE, align="center")

        # Botão RETORNAR/REINICIAR WAYPOINT
        is_navigating = (nav_mode == "route" or nav_mode == "route_finished")
        ret_base_color = BLUE if is_navigating else (MAGENTA if wp_count > 0 else GRAY)
        ret_label = "REINICIAR WYP" if is_navigating else "NAVEGAR WYP"
        
        ret_color = (max(0, ret_base_color[0]-40), max(0, ret_base_color[1]-40), max(0, ret_base_color[2]-40)) \
                    if self.btn_return_wp.collidepoint(mouse_pos) else ret_base_color
        pygame.draw.rect(surface, ret_color, self.btn_return_wp, border_radius=10)
        pygame.draw.rect(surface, WHITE, self.btn_return_wp, 1, border_radius=10)
        self.draw_text(surface, ret_label, self.btn_return_wp.center, self.f_btn, WHITE, align="center")

        # Botão ZERAR WAYPOINTS
        has_wps = wp_count > 0
        clr_base  = (180, 50, 50) if has_wps else (70, 70, 70)
        clr_color = (max(0, clr_base[0]-40), max(0, clr_base[1]-40), max(0, clr_base[2]-40)) \
                    if self.btn_clear_wps.collidepoint(mouse_pos) else clr_base
        pygame.draw.rect(surface, clr_color, self.btn_clear_wps, border_radius=10)
        pygame.draw.rect(surface, WHITE, self.btn_clear_wps, 1, border_radius=10)
        self.draw_text(surface, "ZERAR WYP", self.btn_clear_wps.center, self.f_btn, WHITE, align="center")

        # ── DIVISOR ───────────────────────────────────────────────────────────
        pygame.draw.line(surface, GRAY, (590, 308), (1290, 308), 2)


        # ── ÁREA DE INFORMAÇÕES (abaixo do divisor) ───────────────────────────
        # ── MODO ARREMETIDA - SUBINDO: exibe "MANTER CURSO" no painel central ───
        is_ma_climbing = nav_state.get("missed_approach_climbing", False)
        if is_ma_climbing:
            ai_card_y = 390
            ai_card_rect = pygame.Rect(590, ai_card_y, 700, 330)
            pygame.draw.rect(surface, (50, 30, 0), ai_card_rect, border_radius=15)
            pygame.draw.rect(surface, (255, 165, 0), ai_card_rect, 3, border_radius=15)
            self.draw_text(surface, "ARREMETIDA EM ANDAMENTO", (ai_card_rect.centerx, ai_card_y + 140), self.f_data, (255, 165, 0), align="center")
            self.draw_text(surface, f"MANTER CURSO: {int(course):03}°", (ai_card_rect.centerx, ai_card_y + 190), self.f_data, GREEN, align="center")

        if is_active:
            v_phase = nav_state.get("vectoring_phase", 0)
            
            # Curso travado e desvio XTK
            side = ""
            xtk_color = GREEN
            # Só mostra R ou L se não estiver nas fases de vetoramento inicial (1 a 5)
            if abs(xtk) >= 0.005 and not (1 <= v_phase <= 5):
                side = " R" if xtk > 0 else " L"
            if abs(xtk) > self.xtk_tolerance:
                xtk_color = RED

            # Label de waypoint ativo (modo rota) ou curso manual (modo manual)
            if nav_mode == "route" and waypoints:
                wp_name = waypoints[active_wp_idx].get("name", f"WP{active_wp_idx+1}")
                self.draw_text(surface, "ALVO", (600, 325), self.f_label, WHITE)
                self.draw_text(surface, wp_name, (600, 347), self.f_data, MAGENTA)
            else:
                self.draw_text(surface, "CURSO TRAVADO", (600, 325), self.f_label, WHITE)
                self.draw_text(surface, f"{int(course):03}°", (600, 347), self.f_data, YELLOW)

            # Só mostra XTK em milhas se não estiver nas fases de vetoramento inicial (1 a 5)
            if not (1 <= v_phase <= 5):
                self.draw_text(surface, "DESVIO LATERAL (XTK)", (920, 325), self.f_label, WHITE)
                self.draw_text(surface, f"{abs(xtk):.3f} NM{side}", (920, 347), self.f_data, xtk_color)

            # ── Painel da IA ──────────────────────────────────────────────────
            ai_card_y = 390
            ai_card_rect = pygame.Rect(590, ai_card_y, 700, 330)
            
            v_phase = nav_state.get("vectoring_phase", 0)
            
            if abs(xtk) > self.xtk_tolerance and v_phase != 7:
                pygame.draw.rect(surface, (70, 15, 15), ai_card_rect, border_radius=15)
                pygame.draw.rect(surface, RED, ai_card_rect, 3, border_radius=15)
            else:
                pygame.draw.rect(surface, (25, 20, 45), ai_card_rect, border_radius=15)
                pygame.draw.rect(surface, AI_PURPLE, ai_card_rect, 2, border_radius=15)

            pygame.draw.circle(surface, AI_PURPLE, (630, ai_card_y + 30), 8)
            self.draw_text(surface, "SISTEMA DE VETORAÇÃO GEOMÉTRICA", (645, ai_card_y + 22), self.f_btn, AI_PURPLE)

            heading_error = NavMath.normalize_angle(data["track"] - course)
            is_aligned = abs(heading_error) <= 5.0

            self.draw_text(surface, "PROA DE INTERCEPTAÇÃO SUGERIDA:", (620, ai_card_y + 68), self.f_label, WHITE)
            
            if nav_mode == "route_finished":
                self.draw_text(surface, "---", (620, ai_card_y + 88), self.f_data, GREEN)
            elif v_phase > 0 and v_phase < 7:
                c_side = nav_state.get("circuit_side")
                gs_active = nav_state.get("gs_synthetic_active", False)
                if not c_side:
                    self.draw_text(surface, "IDENTIFICANDO...", (620, ai_card_y + 88), self.f_data, YELLOW)
                else:
                    if v_phase == 1:
                        lado_curva = "DIREITA" if c_side == "R" else "ESQUERDA"
                        self.draw_text(surface, f"CURVAR PARA {lado_curva}", (620, ai_card_y + 88), self.f_data, MAGENTA)
                    elif v_phase == 2 or v_phase == 3:
                        self.draw_text(surface, "MANTER PERNA PARALELA", (620, ai_card_y + 88), self.f_data, BLUE)
                    elif v_phase == 4 or v_phase == 5:
                        base_hdg = (course + 90) % 360 if c_side == "L" else (course - 90 + 360) % 360
                        self.draw_text(surface, f"BASE -> {int(base_hdg):03}°", (620, ai_card_y + 88), self.f_data, MAGENTA)
                    elif v_phase == 6:
                        self.draw_text(surface, f"INTERCEPTAÇÃO -> {int(target_hdg):03}°", (620, ai_card_y + 88), self.f_data, BLUE)
            elif v_phase == 7:
                # Na fase 7, se estiver muito bem alinhado (xtk < tolerância) mostra MANTER CURSO
                if abs(xtk) <= self.xtk_tolerance and is_aligned:
                    self.draw_text(surface, f"MANTER CURSO: {int(course):03}°", (620, ai_card_y + 88), self.f_data, GREEN)
                else:
                    self.draw_text(surface, f"INBOUND: {int(target_hdg):03}°", (620, ai_card_y + 88), self.f_data, BLUE)
            elif abs(xtk) > self.xtk_tolerance or not is_aligned:
                self.draw_text(surface, f"{target_hdg:03}°", (620, ai_card_y + 88), self.f_data, MAGENTA)
            else:
                # Ao invés de ---, mostra o curso desejado grande em verde para o piloto manter
                self.draw_text(surface, f"MANTER CURSO: {int(course):03}°", (620, ai_card_y + 88), self.f_data, GREEN)

            # Texto explicativo da IA + distância à cabeceira (GS Sintético)
            explanation = ""
            gs_active = nav_state.get("gs_synthetic_active", False)
            gs_dist = nav_state.get("gs_dist_to_threshold")
            
            if nav_mode == "route_finished":
                explanation = "IA: Último waypoint finalizado. Sem novos waypoints para monitorar. Rota concluída. Aguardando o recomeço da tarefa."
            elif v_phase > 0 and v_phase < 7:
                c_side = nav_state.get("circuit_side")
                lado = "DIREITA" if c_side == "R" else "ESQUERDA"
                if v_phase == 1:
                    if not c_side:
                        explanation = "SISTEMA: Aguardando curva para direita ou esquerda para estabelecer a perna paralela."
                    else:
                        explanation = f"SISTEMA: Circuito pela {lado} identificado. Curve para estabelecer na perna paralela."
                elif v_phase == 2:
                    explanation = f"SISTEMA: Perna paralela ({lado}). Mantenha proa até cruzar o través do primeiro waypoint."
                elif v_phase == 3:
                    explanation = f"SISTEMA: Través cruzado. Afastando no eixo paralelo ({lado})."
                elif v_phase == 4:
                    explanation = f"SISTEMA: Distância ideal atingida. Inicie curva BASE à {lado} para interceptação."
                elif v_phase == 5:
                    if gs_active:
                        explanation = f"SISTEMA: Través cruzado. Perna BASE estabelecida ({lado}). Siga a proa sugerida de 90° até interceptar o curso."
                    else:
                        explanation = f"SISTEMA: Perna BASE estabelecida ({lado}). Siga a proa sugerida até interceptar o curso."
                elif v_phase == 6:
                    explanation = f"SISTEMA: Interceptando inbound. Siga o Flight Director."
            else:
                if v_phase == 7:
                    explanation = "IA: Aeronave em zona de aproximação. Siga a proa do Inbound dinâmico para interceptar o curso."
                elif abs(xtk) > self.xtk_tolerance:
                    lado_txt = "DIREITA" if xtk > 0 else "ESQUERDA"
                    current_hdg = data.get("heading", 0)
                    diff_hdg = (target_hdg - current_hdg + 180) % 360 - 180
                    correcao_txt = "DIREITA" if diff_hdg > 0 else "ESQUERDA"
                    if nav_mode == "route" and waypoints:
                        wp_name = waypoints[active_wp_idx].get("name", f"WP{active_wp_idx+1}")
                        explanation = (f"IA: Desvio de {abs(xtk):.3f} NM à {lado_txt} da perna até {wp_name}. "
                                       f"Sugiro curva à {correcao_txt} para a proa {target_hdg:03}°.")
                    else:
                        explanation = (f"IA: Identificado desvio lateral de {abs(xtk):.3f} NM à {lado_txt} "
                                       f"(Tolerância excedida: >{self.xtk_tolerance} NM). "
                                       f"Sugiro curva de correção imediata à {correcao_txt} para a proa {target_hdg:03}° "
                                       f"visando interceptar o rumo original.")
                elif not is_aligned:
                    direcao_curva = "ESQUERDA" if heading_error > 0 else "DIREITA"
                    explanation = (f"IA: Aeronave sobre o trilho lateral, mas desalinhada. "
                                   f"Sugiro ajustar rumo à {direcao_curva} para a proa {target_hdg:03}° "
                                   f"para manter o alinhamento da rota.")
                else:
                    if nav_mode == "route" and waypoints:
                        wp_name = waypoints[active_wp_idx].get("name", f"WP{active_wp_idx+1}")
                        explanation = f"IA: Rota estável. Aeronave na perna em direção a {wp_name}. Mantendo parâmetros nominais."
                    else:
                        explanation = f"IA: Rota nominal estável dentro do limite de {self.xtk_tolerance} NM. Aeronave mantendo parâmetros recomendados."
            
            # ── Distância e Razão de Descida até a cabeceira (GS Sintético) ──
            # Exibida sempre que o GS sintético estiver ativo, no canto inferior do painel
            if gs_active and gs_dist is not None:
                dist_color = GREEN if gs_dist <= 10.0 else YELLOW
                dist_txt = f"DIST CABECEIRA: {gs_dist:.1f} NM"
                self.draw_text(surface, dist_txt, (ai_card_rect.centerx, ai_card_y + 300), self.f_btn, dist_color, align="center")

            # Quebra automática de texto
            words = explanation.split(' ')
            lines, current_line = [], ""
            for w in words:
                if len(current_line + w) < 52:
                    current_line += w + " "
                else:
                    lines.append(current_line)
                    current_line = w + " "
            lines.append(current_line)
            
            for index, line in enumerate(lines):
                self.draw_text(surface, line, (620, ai_card_y + 155 + (index * 26)), self.f_msg, WHITE)

        else:
            # Standby ou Ligado sem Rota
            if nav_state.get("system_on", False):
                is_ma_climbing = nav_state.get("missed_approach_climbing", False)
                if not is_ma_climbing:
                    self.draw_text(surface, "SISTEMA INICIADO: AGUARDANDO COMANDOS", (panel_rect.centerx, 370), self.f_btn, GREEN, align="center")
                    self.draw_text(surface, "Digite um curso na caixa e clique em 'SET'", (panel_rect.centerx, 400), self.f_label, LIGHT_GRAY, align="center")
                    self.draw_text(surface, "ou clique em 'NAVEGAR WAYPOINTS' para rota automática.", (panel_rect.centerx, 425), self.f_label, LIGHT_GRAY, align="center")
                    
                    # Painel da IA no modo Standby
                    ai_card_y = 480
                    ai_card_rect = pygame.Rect(590, ai_card_y, 700, 220)
                    pygame.draw.rect(surface, (25, 20, 45), ai_card_rect, border_radius=15)
                    pygame.draw.rect(surface, GRAY, ai_card_rect, 2, border_radius=15)
                    pygame.draw.circle(surface, GRAY, (630, ai_card_y + 30), 8)
                    self.draw_text(surface, "SISTEMA DE VETORAÇÃO (STANDBY)", (645, ai_card_y + 22), self.f_btn, GRAY)
                    self.draw_text(surface, "SISTEMA: Inicializado e monitorando telemetria base.", (620, ai_card_y + 80), self.f_msg, WHITE)
                    self.draw_text(surface, "SISTEMA: Aguardando inserção de curso manual ou ativação de rota.", (620, ai_card_y + 110), self.f_msg, WHITE)

            else:
                self.draw_text(surface, "MONITORAMENTO LATERAL DESLIGADO", (panel_rect.centerx, 390), self.f_btn, GRAY, align="center")
                self.draw_text(surface, "Clique em 'INICIAR MONITOR' para ligar os sistemas.", (panel_rect.centerx, 420), self.f_label, WHITE, align="center")

            if waypoints:
                is_ma_climbing = nav_state.get("missed_approach_climbing", False)
                if not is_ma_climbing:
                    self.draw_text(surface, f"{len(waypoints)} waypoint(s) gravado(s).", (panel_rect.centerx, 720), self.f_label, BLUE, align="center")

    def draw_vnav_panel(self, surface, data, nav_state, mouse_pos, active_input, vnav_alt_text, vnav_tol_text, vnav_cal_text):
        """Desenha o painel de Navegação Vertical (Lado Direito Extra)."""
        panel_rect = pygame.Rect(1340, 24, 440, 852)
        pygame.draw.rect(surface, PANEL_BG, panel_rect, border_radius=14)
        pygame.draw.rect(surface, GRAY, panel_rect, 2, border_radius=14)

        # Título do Painel
        self.draw_text(surface, "NAVEGAÇÃO VERTICAL", (panel_rect.centerx, 50), self.f_title, YELLOW, align="center")

        # Altitude Atual (em destaque)
        current_alt = data.get('alt', 0)
        self.draw_text(surface, "ALTITUDE ATUAL", (panel_rect.centerx, 110), self.f_label, WHITE, align="center")
        self.draw_text(surface, f"{int(current_alt)} FT", (panel_rect.centerx, 155), self.f_data, WHITE, align="center")

        # ── Calibrar Altitude (Offset) ─────────────────────────────
        is_cal_active = (active_input == "vnav_cal")
        cal_border = BLUE if is_cal_active else GRAY
        pygame.draw.rect(surface, BOX_BG, self.vnav_cal_input_rect, border_radius=10)
        pygame.draw.rect(surface, cal_border, self.vnav_cal_input_rect, 2, border_radius=10)
        self.draw_text(surface, "Calibrar ALT (FT):", (self.vnav_cal_input_rect.x, self.vnav_cal_input_rect.y - 20), self.f_small, WHITE)

        display_cal = vnav_cal_text + ("|" if is_cal_active and (pygame.time.get_ticks() // 400) % 2 == 0 else "")
        if not vnav_cal_text and not is_cal_active:
            self.draw_text(surface, "ex: 2100", (self.vnav_cal_input_rect.x + 12, self.vnav_cal_input_rect.y + 11), self.f_small, GRAY)
        else:
            self.draw_text(surface, display_cal, (self.vnav_cal_input_rect.x + 12, self.vnav_cal_input_rect.y + 11), self.f_btn, WHITE)
        
        # Botão Ativar/Desativar Offset
        is_calibrated = nav_state.get("alt_calibrated", False)
        toggle_color = GREEN if is_calibrated else (80, 80, 80)
        if self.btn_toggle_vnav_cal.collidepoint(mouse_pos):
            toggle_color = (min(255, toggle_color[0]+30), min(255, toggle_color[1]+30), min(255, toggle_color[2]+30))
        pygame.draw.rect(surface, toggle_color, self.btn_toggle_vnav_cal, border_radius=10)
        pygame.draw.rect(surface, WHITE, self.btn_toggle_vnav_cal, 1, border_radius=10)
        toggle_txt = "ON" if is_calibrated else "OFF"
        self.draw_text(surface, toggle_txt, self.btn_toggle_vnav_cal.center, self.f_btn, WHITE, align="center")

        # ── Altitude Desejada ──────────────────────────────────────
        is_alt_active = (active_input == "vnav_alt")
        alt_border = BLUE if is_alt_active else GRAY
        pygame.draw.rect(surface, BOX_BG, self.vnav_alt_input_rect, border_radius=10)
        pygame.draw.rect(surface, alt_border, self.vnav_alt_input_rect, 2, border_radius=10)
        self.draw_text(surface, "Altitude Desejada (FT):", (self.vnav_alt_input_rect.x, self.vnav_alt_input_rect.y - 20), self.f_label, WHITE)

        display_alt = vnav_alt_text + ("|" if is_alt_active and (pygame.time.get_ticks() // 400) % 2 == 0 else "")
        if not vnav_alt_text and not is_alt_active:
            val = nav_state.get("vnav_target_alt")
            txt = f"ex: {int(val)}" if val is not None else "ex: 5000"
            self.draw_text(surface, txt, (self.vnav_alt_input_rect.x + 12, self.vnav_alt_input_rect.y + 11), self.f_label, GRAY)
        else:
            self.draw_text(surface, display_alt, (self.vnav_alt_input_rect.x + 12, self.vnav_alt_input_rect.y + 8), self.f_btn, WHITE)

        btn_alt_color = (100, 100, 105) if self.btn_set_vnav_alt.collidepoint(mouse_pos) else GRAY
        pygame.draw.rect(surface, btn_alt_color, self.btn_set_vnav_alt, border_radius=10)
        pygame.draw.rect(surface, WHITE, self.btn_set_vnav_alt, 1, border_radius=10)
        self.draw_text(surface, "SET", self.btn_set_vnav_alt.center, self.f_btn, WHITE, align="center")

        # ── Variação Aceita ────────────────────────────────────────
        is_tol_active = (active_input == "vnav_tol")
        tol_border = BLUE if is_tol_active else GRAY
        pygame.draw.rect(surface, BOX_BG, self.vnav_tol_input_rect, border_radius=10)
        pygame.draw.rect(surface, tol_border, self.vnav_tol_input_rect, 2, border_radius=10)
        self.draw_text(surface, "Variação Aceita (+/- FT):", (self.vnav_tol_input_rect.x, self.vnav_tol_input_rect.y - 20), self.f_label, WHITE)

        display_tol = vnav_tol_text + ("|" if is_tol_active and (pygame.time.get_ticks() // 400) % 2 == 0 else "")
        if not vnav_tol_text and not is_tol_active:
            val = nav_state.get("vnav_tolerance", 200)
            self.draw_text(surface, f"ex: {int(val)}", (self.vnav_tol_input_rect.x + 12, self.vnav_tol_input_rect.y + 11), self.f_label, GRAY)
        else:
            self.draw_text(surface, display_tol, (self.vnav_tol_input_rect.x + 12, self.vnav_tol_input_rect.y + 8), self.f_btn, WHITE)

        btn_tol_color = (100, 100, 105) if self.btn_set_vnav_tol.collidepoint(mouse_pos) else GRAY
        pygame.draw.rect(surface, btn_tol_color, self.btn_set_vnav_tol, border_radius=10)
        pygame.draw.rect(surface, WHITE, self.btn_set_vnav_tol, 1, border_radius=10)
        self.draw_text(surface, "SET", self.btn_set_vnav_tol.center, self.f_btn, WHITE, align="center")

        # ── STATUS DO VNAV ─────────────────────────────────────────
        target_alt = nav_state.get("vnav_target_alt")
        tolerance = nav_state.get("vnav_tolerance", 200)
        status_box = pygame.Rect(1380, 430, 360, 150)
        v_phase_vnav = nav_state.get("vectoring_phase", 0)
        is_ma_climbing = nav_state.get("missed_approach_climbing", False)

        # ── Modo Arremetida: exibe indicador de subida até 4000 ft ──
        if is_ma_climbing:
            ma_target = nav_state.get("missed_approach_target_alt", 4000)
            ft_remaining = max(0, ma_target - current_alt)
            # Caixa laranja pulsante durante a subida
            pulse = (pygame.time.get_ticks() // 500) % 2 == 0
            box_color = (90, 50, 0) if pulse else (70, 35, 0)
            border_color = (255, 165, 0) if pulse else (200, 120, 0)  # Laranja
            pygame.draw.rect(surface, box_color, status_box, border_radius=15)
            pygame.draw.rect(surface, border_color, status_box, 3, border_radius=15)
            self.draw_text(surface, "ARREMETIDA - SUBINDO", (status_box.centerx, status_box.y + 35), self.f_title, (255, 165, 0), align="center")
            self.draw_text(surface, f"META: {int(ma_target)} FT", (status_box.centerx, status_box.y + 75), self.f_btn, WHITE, align="center")
            self.draw_text(surface, f"FALTAM: {int(ft_remaining)} FT", (status_box.centerx, status_box.y + 115), self.f_btn, (255, 200, 100), align="center")

        # Exibe status VNAV normal apenas fora das fases de vetoramento inicial (1 a 5)
        elif target_alt is not None and not (1 <= v_phase_vnav <= 5):
            diff = current_alt - target_alt
            abs_diff = abs(diff)

            if abs_diff > tolerance:
                # Alerta Vermelho Fixo
                pygame.draw.rect(surface, (70, 15, 15), status_box, border_radius=15)
                pygame.draw.rect(surface, RED, status_box, 3, border_radius=15)
                self.draw_text(surface, "ALERTA DE ALTITUDE", (status_box.centerx, status_box.y + 40), self.f_title, RED, align="center")
                direcao = "ABAIXO" if diff < 0 else "ACIMA"
                self.draw_text(surface, f"{int(abs_diff)} FT {direcao} DO ALVO", (status_box.centerx, status_box.y + 90), self.f_btn, WHITE, align="center")
            else:
                # Status OK (Verde)
                pygame.draw.rect(surface, (20, 45, 25), status_box, border_radius=15)
                pygame.draw.rect(surface, GREEN, status_box, 2, border_radius=15)
                self.draw_text(surface, "ALTITUDE MANTIDA", (status_box.centerx, status_box.y + 40), self.f_title, GREEN, align="center")
                self.draw_text(surface, f"ALVO: {int(target_alt)} FT", (status_box.centerx, status_box.y + 90), self.f_btn, WHITE, align="center")
        else:
            # Standby
            pygame.draw.rect(surface, BOX_BG, status_box, border_radius=15)
            pygame.draw.rect(surface, GRAY, status_box, 2, border_radius=15)
            self.draw_text(surface, "VNAV STANDBY", (status_box.centerx, status_box.centery), self.f_title, GRAY, align="center")

    def draw_procedure_panel(self, surface, mouse_pos, selected_chart_name, available_charts, show_dropdown=False, nav_state=None, show_dir_dropdown=False):
        """Desenha o painel de Seleção de Procedimentos (Cartas) na parte inferior direita."""
        # A área disponível no painel direito (x=1340, w=440) abaixo do y=600
        proc_rect = pygame.Rect(1360, 600, 400, 280)
        pygame.draw.rect(surface, BOX_BG, proc_rect, border_radius=14)
        pygame.draw.rect(surface, GRAY, proc_rect, 2, border_radius=14)

        self.draw_text(surface, "MONITOR DE PROCEDIMENTO", (proc_rect.centerx, 615), self.f_title, GREEN, align="center")
        
        # Botão para abrir o dropdown
        self.btn_cycle_chart = pygame.Rect(1380, 640, 360, 40)
        btn_cycle_color = (100, 100, 105) if self.btn_cycle_chart.collidepoint(mouse_pos) else GRAY
        pygame.draw.rect(surface, btn_cycle_color, self.btn_cycle_chart, border_radius=10)
        pygame.draw.rect(surface, WHITE, self.btn_cycle_chart, 1, border_radius=10)
        
        display_name = selected_chart_name if selected_chart_name else "NENHUMA (MANUAL)"
        self.draw_text(surface, f"CARTA: {display_name}", (self.btn_cycle_chart.x + 10, self.btn_cycle_chart.y + 8), self.f_btn, WHITE)
        
        # Desenha uma setinha indicando dropdown
        arrow_x = self.btn_cycle_chart.right - 20
        arrow_y = self.btn_cycle_chart.centery
        if show_dropdown:
            pygame.draw.polygon(surface, WHITE, [(arrow_x-6, arrow_y+3), (arrow_x+6, arrow_y+3), (arrow_x, arrow_y-4)])
        else:
            pygame.draw.polygon(surface, WHITE, [(arrow_x-6, arrow_y-3), (arrow_x+6, arrow_y-3), (arrow_x, arrow_y+4)])

        # Botão para aplicar a carta
        self.btn_load_chart = pygame.Rect(1380, 690, 360, 40)
        btn_load_color = BLUE if selected_chart_name else GRAY
        if self.btn_load_chart.collidepoint(mouse_pos) and selected_chart_name:
            btn_load_color = (max(0, btn_load_color[0]-40), max(0, btn_load_color[1]-40), max(0, btn_load_color[2]-40))
        
        pygame.draw.rect(surface, btn_load_color, self.btn_load_chart, border_radius=10)
        pygame.draw.rect(surface, WHITE, self.btn_load_chart, 1, border_radius=10)
        self.draw_text(surface, "CARREGAR PROCEDIMENTO", self.btn_load_chart.center, self.f_btn, WHITE, align="center")

        # Botões de Operação do Procedimento (Aparecem apenas quando navegando uma carta)
        self.btn_toga = None
        self.btn_dir = None
        dir_dropdown_rects = []
        is_navigating_proc = nav_state and nav_state.get("navigation_mode") == "route" and nav_state.get("active_procedure")
        
        if is_navigating_proc:
            # Botão DIRECT-TO
            self.btn_dir = pygame.Rect(1380, 740, 360, 40)
            dir_color = MAGENTA
            if self.btn_dir.collidepoint(mouse_pos):
                dir_color = (max(0, dir_color[0]-40), max(0, dir_color[1]-40), max(0, dir_color[2]-40))
            pygame.draw.rect(surface, dir_color, self.btn_dir, border_radius=10)
            pygame.draw.rect(surface, WHITE, self.btn_dir, 1, border_radius=10)
            self.draw_text(surface, "IR DIRETO P/ FIXO (DIR->)", self.btn_dir.center, self.f_btn, WHITE, align="center")
            
            # Setinha do dropdown DIR->
            arrow_x = self.btn_dir.right - 20
            arrow_y = self.btn_dir.centery
            if show_dir_dropdown:
                pygame.draw.polygon(surface, WHITE, [(arrow_x-6, arrow_y+3), (arrow_x+6, arrow_y+3), (arrow_x, arrow_y-4)])
            else:
                pygame.draw.polygon(surface, WHITE, [(arrow_x-6, arrow_y-3), (arrow_x+6, arrow_y-3), (arrow_x, arrow_y+4)])

            # Botão TOGA
            self.btn_toga = pygame.Rect(1380, 790, 360, 40)
            toga_color = RED
            if self.btn_toga.collidepoint(mouse_pos):
                toga_color = (max(0, toga_color[0]-40), max(0, toga_color[1]-40), max(0, toga_color[2]-40))
            pygame.draw.rect(surface, toga_color, self.btn_toga, border_radius=10)
            pygame.draw.rect(surface, WHITE, self.btn_toga, 1, border_radius=10)
            self.draw_text(surface, "ARREMETER (TOGA)", self.btn_toga.center, self.f_btn, WHITE, align="center")
        else:
            self.draw_text(surface, "Arremetida com fuga automática p/ MAHF.", (proc_rect.centerx, 825), self.f_label, LIGHT_GRAY, align="center")

        # Desenha o Dropdown de Cartas por cima de tudo
        dropdown_rects = []
        if show_dropdown:
            options = ["NENHUMA (MANUAL)"] + available_charts
            drop_h = len(options) * 40
            drop_rect = pygame.Rect(1380, self.btn_cycle_chart.bottom + 2, 360, drop_h)
            pygame.draw.rect(surface, (20, 20, 25), drop_rect, border_radius=10)
            pygame.draw.rect(surface, WHITE, drop_rect, 2, border_radius=10)
            
            for i, opt in enumerate(options):
                opt_rect = pygame.Rect(1380, drop_rect.y + i * 40, 360, 40)
                dropdown_rects.append((opt, opt_rect))
                if opt_rect.collidepoint(mouse_pos):
                    pygame.draw.rect(surface, (60, 60, 80), opt_rect, border_radius=10)
                color = YELLOW if (opt == selected_chart_name or (opt == "NENHUMA (MANUAL)" and not selected_chart_name)) else WHITE
                self.draw_text(surface, opt, (1395, drop_rect.y + i * 40 + 8), self.f_msg, color)
                
        # Desenha o Dropdown DIR-> por cima de tudo
        if show_dir_dropdown and is_navigating_proc:
            wps = nav_state.get("recorded_waypoints", [])
            options = [wp["name"] for wp in wps]
            drop_h = len(options) * 40
            drop_rect = pygame.Rect(1380, self.btn_dir.bottom + 2, 360, drop_h)
            pygame.draw.rect(surface, (20, 20, 25), drop_rect, border_radius=10)
            pygame.draw.rect(surface, WHITE, drop_rect, 2, border_radius=10)
            
            for i, opt in enumerate(options):
                opt_rect = pygame.Rect(1380, drop_rect.y + i * 40, 360, 40)
                dir_dropdown_rects.append((i, opt_rect))  # Retorna o index
                if opt_rect.collidepoint(mouse_pos):
                    pygame.draw.rect(surface, (60, 60, 80), opt_rect, border_radius=10)
                color = YELLOW if i == nav_state.get("active_waypoint_index") else WHITE
                self.draw_text(surface, f"DIR -> {opt}", (1395, drop_rect.y + i * 40 + 8), self.f_msg, color)

        return dropdown_rects, dir_dropdown_rects



import sys
import argparse
import pygame
import time
import math


# Caixas de Dados de Telemetria (HDG, TRK, SPD, ALT)
HDG_RECT = pygame.Rect(94,  22, 165, 82)
TRK_RECT = pygame.Rect(282, 22, 165, 82)
SPD_RECT = pygame.Rect(0,  338,  86, 86)
ALT_RECT = pygame.Rect(470, 338,  86, 86)


def get_scaled_view_rect(window_size):
    """Calcula a proporção e enquadramento da tela para manter o aspecto original."""
    base_width, base_height = BASE_WINDOW_SIZE
    window_width, window_height = window_size
    scale = min(window_width / base_width, window_height / base_height)
    scaled_width = int(base_width * scale)
    scaled_height = int(base_height * scale)
    offset_x = (window_width - scaled_width) // 2
    offset_y = (window_height - scaled_height) // 2
    return pygame.Rect(offset_x, offset_y, scaled_width, scaled_height), scale


def window_to_canvas_pos(pos, window_size):
    """Converte a coordenada do mouse da janela física para as coordenadas internas do canvas."""
    view_rect, scale = get_scaled_view_rect(window_size)
    if scale <= 0:
        return 0, 0
    x = int((pos[0] - view_rect.x) / scale)
    y = int((pos[1] - view_rect.y) / scale)
    return x, y


def draw_scaled_canvas(screen, canvas, window_size):
    """Desenha o canvas interno escalado suavemente na janela física do Pygame."""
    view_rect, _ = get_scaled_view_rect(window_size)
    screen.fill((0, 0, 0))
    scaled_canvas = pygame.transform.smoothscale(canvas, view_rect.size)
    screen.blit(scaled_canvas, view_rect)


def main():
    # Parsing de argumentos para selecionar a fonte de dados
    parser = argparse.ArgumentParser(description="Monitor de Navegação Lateral")
    parser.add_argument("--mode", type=str, choices=["sim", "pi", "mock"], default="auto",
                        help="Seleciona a fonte de telemetria: 'sim' (MSFS), 'pi' (Sensores Raspberry Pi), 'mock' (Simulado)")
    args = parser.parse_args()

    # Seleção automática da fonte
    source = None
    mode_chosen = args.mode

    if mode_chosen == "auto":
        if SIMCONNECT_AVAILABLE:
            mode_chosen = "sim"
        elif PI_SENSORS_AVAILABLE:
            mode_chosen = "pi"
        else:
            mode_chosen = "mock"

    print(f"Modo de telemetria selecionado: {mode_chosen.upper()}")

    if mode_chosen == "sim":
        source = SimConnectSource()
    elif mode_chosen == "pi":
        # Porta padrão para o GPS Neo M8N na Pi
        source = PiSensorSource(gps_port="/dev/ttyAMA0", gps_baud=9600)
    else:
        source = MockSource()

    # Tenta conectar à fonte de dados
    connected = source.connect()
    if not connected and mode_chosen != "mock":
        print("Falha ao inicializar fonte selecionada. Iniciando modo MOCK simulado...")
        source = MockSource()
        source.connect()
        mode_chosen = "mock"

    # Inicialização do Pygame
    pygame.init()
    screen = pygame.display.set_mode(WINDOW_SIZE, pygame.RESIZABLE)
    canvas = pygame.Surface(BASE_WINDOW_SIZE)
    window_size = WINDOW_SIZE
    pygame.display.set_caption(f"Naveg-AI Monitor - Telemetria ({mode_chosen.upper()})")

    # Fontes básicas para as caixas do PFD
    box_title_font = pygame.font.SysFont("Segoe UI", 20, bold=True)
    box_value_font = pygame.font.SysFont("Consolas", 36, bold=True)
    
    clock = pygame.time.Clock()
    gui = PFDGui(base_size=BASE_WINDOW_SIZE)

    # Estado de Navegação Lateral
    # Todos os blocos do código foram totalmente comentados.
    nav_state = {
        "system_on": False,         # Sistema ligado (processando IA e botões), mas não necessariamente rastreando.
        "active": False,            # Monitoramento lateral ativo (curso travado).
        "lat_orig": 0.0,
        "lon_orig": 0.0,
        "course": 0.0,              # Curso magnético exibido na interface (ex: 172°)
        "course_true": 0.0,         # Curso verdadeiro usado nos cálculos trigonométricos
        "xtk": 0.0,                 # Cross-track error calculado em milhas náuticas (NM)
        "target_heading": 0,        # Proa magnética sugerida pela IA de correção
        "xtk_tolerance": 0.3,       # Tolerância de desvio lateral em milhas náuticas (padrão: 0.3 NM)
        # ── Campos de Waypoints ──────────────────────────────────────────────
        "recorded_waypoints": [],   # Lista de dicts: {name, lat, lon, course, course_true}
        "navigation_mode": "manual",# 'manual' ou 'route'
        "active_waypoint_index": 0, # Índice do waypoint alvo atual no modo 'route'
        # ── Campos de Vetoração (Procedural Turn) ─────────────────────────────
        "vectoring_phase": 0,           # 0: Inativo, 1: Aguardando curva, 2: Perna de afastamento, 3: Segunda curva
        "vectoring_initial_heading": 0.0,
        "vectoring_turn_dir": 1,        # 1: Direita, -1: Esquerda
        "vectoring_target_hdg": 0.0,
        "vnav_target_alt": None,
        "vnav_tolerance": 200,
        "prev_dist_to_wp": None,
        "alt_offset": 0.0,
        "alt_calibrated": False,
        # ── Arremetida ─────────────────────────────────────────────────────────
        "missed_approach_climbing": False,  # True: passou da cabeceira, subindo para 4000 ft
        "missed_approach_target_alt": 4000, # Altitude alvo para reativar o FD na arremetida
        # ── Glide Slope Sintético ──────────────────────────────────────────────
        "gs_synthetic_active": False,       # True quando a rampa sintética de 3° está ativa
        "gs_rwy_elev": None,                # Elevação da pista digitada pelo usuário (FT)
        "gs_thr_lat": None,                 # Latitude da cabeceira (posição da aeronave no clique INICIAR)
        "gs_thr_lon": None,                 # Longitude da cabeceira
        "gs_inbound_course": None,          # Curso magnético de aproximação (rumo para a pista)
        "gs_inbound_course_true": None,     # Curso verdadeiro de aproximação
        "gs_dist_to_threshold": None,       # Distância atual até a cabeceira (calculada em tempo real)
    }

    # Estado da entrada de dados na interface (três caixas de texto com foco dinâmico)
    active_input = "none"       # Foco ativo: "course", "tolerance", "timer", "vnav_alt", "vnav_tol", "vnav_cal", "gs_rwy_alt" ou "none"
    course_text = ""            # Buffer para digitação do curso
    tolerance_text = "0.3"      # Buffer para digitação da tolerância (iniciado com o padrão 0.3)
    vnav_alt_text = ""          # Buffer para digitação da altitude VNAV
    vnav_tol_text = ""          # Buffer para digitação da tolerância VNAV
    vnav_cal_text = ""          # Buffer para digitação do Offset de Altitude
    gs_rwy_alt_text = ""        # Buffer para digitação da altitude da pista (Glide Slope Sintético)
    mouse_pos = (0, 0)

    # Contador para nomeação automática dos waypoints (WYP01, WYP02, ...)
    wp_counter = 0

    # Informações extras de ajuda no console para o modo Mock
    if mode_chosen == "mock":
        print("\n=== ATALHOS DO TECLADO (MODO MOCK) ===")
        print("Pressione [R] para simular DERIVA lateral à DIREITA (vento lateral)")
        print("Pressione [L] para simular DERIVA lateral à ESQUERDA (vento lateral)")
        print("Pressione [C] para CANCELAR a deriva (voar reto)")
        print("======================================\n")

    # Estado de Procedimentos (Cartas)
    available_chart_keys = list(AVAILABLE_ROUTES.keys())
    selected_chart_index = -1
    selected_chart_name = None
    show_chart_dropdown = False
    show_dir_dropdown = False

    running = True
    while running:
        # Aquisição de dados de telemetria
        data = source.get_data()
        
        # Aplica o offset de calibração se estiver ativo
        if nav_state.get("alt_calibrated"):
            data["alt"] += nav_state.get("alt_offset", 0.0)
            
        # Gerenciamento de eventos
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.VIDEORESIZE:
                window_size = event.size
                screen = pygame.display.set_mode(window_size, pygame.RESIZABLE)

            elif event.type == pygame.MOUSEMOTION:
                mouse_pos = window_to_canvas_pos(event.pos, window_size)

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = window_to_canvas_pos(event.pos, window_size)
                if event.button == 1:
                    # Foca na caixa clicada ou remove foco
                    if gui.course_input_rect.collidepoint(mouse_pos):
                        active_input = "course"
                    elif gui.tolerance_input_rect.collidepoint(mouse_pos):
                        active_input = "tolerance"
                    elif gui.vnav_alt_input_rect.collidepoint(mouse_pos):
                        active_input = "vnav_alt"
                    elif gui.vnav_tol_input_rect.collidepoint(mouse_pos):
                        active_input = "vnav_tol"
                    elif gui.vnav_cal_input_rect.collidepoint(mouse_pos):
                        active_input = "vnav_cal"
                    elif gui.gs_rwy_alt_input_rect.collidepoint(mouse_pos):
                        active_input = "gs_rwy_alt"
                    else:
                        active_input = "none"
                
                # Clique no botão SET do Curso desejado
                if gui.btn_set_course.collidepoint(mouse_pos):
                    if course_text.isdigit():
                        course_val = int(course_text) % 360
                        if data.get("connected"):
                            # Só captura novas coordenadas de origem se nenhuma estiver definida (ex: 0.0)
                            # Caso contrário, mantém a origem original para calcular o desvio acumulado corretamente
                            if nav_state["lat_orig"] == 0.0 and nav_state["lon_orig"] == 0.0:
                                nav_state["lat_orig"] = data["lat"]
                                nav_state["lon_orig"] = data["lon"]
                            nav_state["course"] = float(course_val)
                            
                            # Converte o curso manual (que é magnético) para verdadeiro (True) usando mag_var
                            mag_var = data.get("mag_var", 0.0)
                            nav_state["course_true"] = (float(course_val) + mag_var) % 360
                            nav_state["active"] = True
                            print(f"Curso de Navegação Travado Manualmente: {course_val:03}° (True: {int(nav_state['course_true']):03}°)")
                        active_input = "none"

                # Clique no botão SET da Tolerância XTK customizada
                if gui.btn_set_tolerance.collidepoint(mouse_pos):
                    try:
                        # Permite floats (ex: 0.15) ou inteiros (ex: 1)
                        tolerance_val = float(tolerance_text) if tolerance_text else 0.3
                        if tolerance_val <= 0:
                            tolerance_val = 0.3
                        nav_state["xtk_tolerance"] = tolerance_val
                        print(f"Tolerância XTK atualizada para: {tolerance_val} NM")
                    except ValueError:
                        print("Valor de tolerância inválido. Mantendo o anterior.")
                    active_input = "none"

                # Clique no botão SET VNAV ALT
                if gui.btn_set_vnav_alt.collidepoint(mouse_pos):
                    try:
                        if vnav_alt_text:
                            nav_state["vnav_target_alt"] = float(vnav_alt_text)
                            print(f"Altitude Alvo VNAV atualizada para: {nav_state['vnav_target_alt']} FT")
                    except ValueError:
                        print("Altitude VNAV inválida.")
                    active_input = "none"

                # Clique no botão SET VNAV TOL
                if gui.btn_set_vnav_tol.collidepoint(mouse_pos):
                    try:
                        tol_val = float(vnav_tol_text) if vnav_tol_text else 200.0
                        if tol_val < 0:
                            tol_val = 200.0
                        nav_state["vnav_tolerance"] = tol_val
                        print(f"Tolerância VNAV atualizada para: {tol_val} FT")
                    except ValueError:
                        print("Tolerância VNAV inválida.")
                    active_input = "none"

                # Clique no botão TOGGLE VNAV CAL
                if hasattr(gui, 'btn_toggle_vnav_cal') and gui.btn_toggle_vnav_cal.collidepoint(mouse_pos):
                    is_currently_on = nav_state.get("alt_calibrated", False)
                    if not is_currently_on:
                        # Liga a calibração
                        try:
                            if vnav_cal_text:
                                desired_alt = float(vnav_cal_text)
                                raw_alt = source.get_data().get("alt", 0)
                                offset = desired_alt - raw_alt
                                nav_state["alt_offset"] = offset
                                nav_state["alt_calibrated"] = True
                                print(f"Altitude Calibrada (ON)! Desejada: {desired_alt} | Raw: {raw_alt} | Offset: {offset}")
                            else:
                                print("Digite uma altitude antes de ativar a calibração.")
                        except ValueError:
                            print("Valor de calibração inválido.")
                    else:
                        # Desliga a calibração
                        nav_state["alt_calibrated"] = False
                        nav_state["alt_offset"] = 0.0
                        print("Calibração de Altitude (Offset): DESLIGADO")
                    active_input = "none"

                # Clique no botão INICIAR 50FT (Glide Slope Sintético)
                if hasattr(gui, 'btn_start_gs') and gui.btn_start_gs.collidepoint(mouse_pos):
                    # Toggle: se já estiver ativo, desativa
                    if nav_state.get("gs_synthetic_active"):
                        nav_state["gs_synthetic_active"] = False
                        nav_state["gs_rwy_elev"] = None
                        nav_state["gs_thr_lat"] = None
                        nav_state["gs_thr_lon"] = None
                        nav_state["gs_dist_to_threshold"] = None
                        nav_state["recorded_waypoints"] = []
                        nav_state["navigation_mode"] = "manual"
                        nav_state["active_waypoint_index"] = 0
                        nav_state["active"] = False
                        nav_state["vectoring_phase"] = 0
                        nav_state["vnav_gs_alt"] = None
                        nav_state["vnav_target_alt"] = None
                        wp_counter = 0
                        course_text = ""
                        vnav_alt_text = ""
                        print("Glide Slope Sintético DESATIVADO.")
                    else:
                        # Ativação: valida altitude da pista e curso
                        try:
                            if not data.get("connected"):
                                print("ERRO: Sem conexão com sensores. Impossível iniciar o GS Sintético.")
                            else:
                                # Se a caixa de altitude estiver vazia, puxa a altitude atual do avião (já que está no chão)
                                if gs_rwy_alt_text:
                                    rwy_elev = float(gs_rwy_alt_text)
                                else:
                                    rwy_elev = float(data.get("alt", 0.0))
                                    gs_rwy_alt_text = str(int(rwy_elev))  # Preenche a caixinha automaticamente na tela
                                    
                                thr_lat = data["lat"]
                                thr_lon = data["lon"]
                                mag_var = data.get("mag_var", 0.0)
                                
                                # Puxa automaticamente a proa atual como curso da pista
                                inbound_mag = data.get("heading", 0.0)
                                inbound_true = (inbound_mag + mag_var) % 360
                                
                                # Atualiza as variáveis de interface para exibir o curso puxado
                                nav_state["course"] = inbound_mag
                                nav_state["course_true"] = inbound_true
                                course_text = f"{int(inbound_mag):03}"
                                
                                # Gera os 10 waypoints da rampa de GS
                                gs_wps = NavMath.generate_gs_waypoints(
                                    rwy_thr_lat=thr_lat,
                                    rwy_thr_lon=thr_lon,
                                    inbound_course_true=inbound_true,
                                    inbound_course_mag=inbound_mag,
                                    rwy_elev_ft=rwy_elev,
                                    num_points=10,
                                    max_dist_nm=10.0,
                                    gs_deg=3.0,
                                    tch_ft=50
                                )
                                
                                # Armazena no state
                                nav_state["gs_synthetic_active"] = True
                                nav_state["gs_rwy_elev"] = rwy_elev
                                nav_state["gs_thr_lat"] = thr_lat
                                nav_state["gs_thr_lon"] = thr_lon
                                nav_state["gs_inbound_course"] = inbound_mag
                                nav_state["gs_inbound_course_true"] = inbound_true
                                
                                # Carrega os waypoints e ativa a navegação por rota
                                nav_state["recorded_waypoints"] = gs_wps
                                nav_state["active_waypoint_index"] = 0
                                nav_state["navigation_mode"] = "route"
                                nav_state["system_on"] = True
                                nav_state["active"] = True
                                nav_state["vectoring_phase"] = 1  # Aguardando curva de saída (paralela)
                                nav_state["circuit_side"] = None
                                nav_state["vectoring_initial_heading"] = data["heading"]
                                nav_state["active_procedure"] = None  # Não é procedimento formal
                                wp_counter = len(gs_wps)
                                
                                # Define o VNAV para o primeiro WP (mais afastado = mais alto)
                                first_wp = gs_wps[0]
                                nav_state["vnav_target_alt"] = first_wp.get("alt")
                                vnav_alt_text = str(int(first_wp.get("alt", 0)))
                                
                                # O curso da navegação passa a ser o inbound (eixo da pista)
                                nav_state["lat_orig"] = data["lat"]
                                nav_state["lon_orig"] = data["lon"]
                                nav_state["course_true"] = inbound_true
                                nav_state["course"] = inbound_mag
                                course_text = f"{int(inbound_mag):03}"
                                
                                print(f"✈ Glide Slope Sintético ATIVADO!")
                                print(f"  Cabeceira: {thr_lat:.5f}, {thr_lon:.5f}")
                                print(f"  Elevação pista: {rwy_elev} FT | TCH: {rwy_elev+50:.0f} FT")
                                print(f"  Inbound: {int(inbound_mag):03}° Mag | {int(inbound_true):03}° True")
                                print(f"  {len(gs_wps)} waypoints gerados:")
                                for wp in gs_wps:
                                    print(f"    {wp['name']}: {wp['lat']:.5f}, {wp['lon']:.5f} | ALT={wp['alt']:.0f} FT")
                        except ValueError:
                            print("ERRO: Altitude de pista inválida. Use apenas números inteiros (ex: 2100).")
                    active_input = "none"

                # Clique no botão INICIAR/PARAR MONITOR
                if gui.btn_monitor.collidepoint(mouse_pos):
                    if nav_state["system_on"]:
                        # Parar monitor desliga o sistema e cancela rotas
                        nav_state["system_on"] = False
                        nav_state["active"] = False
                        nav_state["navigation_mode"] = "manual"
                        nav_state["active_waypoint_index"] = 0
                        nav_state["vectoring_phase"] = 0
                        print("Monitoramento de rota e sistema desligados.")
                    else:
                        if data.get("connected"):
                            nav_state["system_on"] = True
                            print("Sistema Iniciado! Aguardando inserção de curso manual ou ativação de rota.")

                # Clique no botão GRAVAR WAYPOINT
                if gui.btn_record_wp.collidepoint(mouse_pos):
                    wps = nav_state["recorded_waypoints"]
                    if len(wps) < 10 and data.get("connected"):
                        wp_counter += 1
                        wp_name = f"WYP{wp_counter:02d}"
                        mag_var = data.get("mag_var", 0.0)
                        # Usa o curso atual: se há curso SET usa-o; senão usa o track atual
                        cur_course = nav_state["course"] if nav_state["active"] else data["track"]
                        cur_course_true = nav_state["course_true"] if nav_state["active"] else data["true_track"]
                        new_wp = {
                            "name": wp_name,
                            "lat": data["lat"],
                            "lon": data["lon"],
                            "alt": data.get("alt", 0),
                            "course": cur_course,
                            "course_true": cur_course_true
                        }
                        wps.append(new_wp)
                        print(f"Waypoint gravado: {wp_name} | Lat={data['lat']:.5f} Lon={data['lon']:.5f} | Curso={int(cur_course):03}°")
                    elif not data.get("connected"):
                        print("Sem conexão: impossível gravar waypoint.")
                    else:
                        print("Limite de 10 waypoints atingido.")

                # Clique no botão NAVEGAR WAYPOINTS / REINICIAR WAYPOINTS
                if gui.btn_return_wp.collidepoint(mouse_pos):
                    wps = nav_state["recorded_waypoints"]
                    if len(wps) > 0 and data.get("connected"):
                        nav_state["navigation_mode"] = "route"
                        nav_state["active_waypoint_index"] = 0
                        nav_state["active"] = True
                        nav_state["system_on"] = True
                        nav_state["circuit_side"] = None
                        
                        first_wp = wps[0]
                        
                        if nav_state.get("gs_synthetic_active"):
                            # No modo GS Sintético, não fazemos Direct-To! O eixo da pista é sagrado.
                            # Restauramos a origem para a cabeceira e mantemos o curso original da pista.
                            nav_state["lat_orig"] = nav_state.get("gs_thr_lat", nav_state["lat_orig"])
                            nav_state["lon_orig"] = nav_state.get("gs_thr_lon", nav_state["lon_orig"])
                            nav_state["course"] = nav_state.get("gs_inbound_course", nav_state["course"])
                            nav_state["course_true"] = nav_state.get("gs_inbound_course_true", nav_state["course_true"])
                            course_text = f"{int(nav_state['course']):03}"
                        else:
                            # Modos Manuais/Procedimentos: A primeira perna é um Direct-To da posição atual
                            bearing_true = NavMath.bearing_degrees(data["lat"], data["lon"], first_wp["lat"], first_wp["lon"])
                            mag_var = data.get("mag_var", 0.0)
                            bearing_mag = (bearing_true - mag_var + 360) % 360
                            
                            nav_state["course_true"] = bearing_true
                            nav_state["course"] = bearing_mag
                            nav_state["lat_orig"] = data["lat"]
                            nav_state["lon_orig"] = data["lon"]
                            course_text = f"{int(bearing_mag):03}"
                        
                        # Carrega também a altitude do WP para o VNAV automaticamente
                        nav_state["vnav_target_alt"] = first_wp.get("alt")
                        vnav_alt_text = str(int(first_wp.get("alt", 0))) if first_wp.get("alt") else ""
                        
                        # Inicia a vetoração se não for um procedimento formal
                        if nav_state.get("active_procedure"):
                            nav_state["vectoring_phase"] = 0
                            print(f"Modo procedimento (re)iniciado para {first_wp['name']}.")
                        else:
                            nav_state["vectoring_phase"] = 1
                            nav_state["vectoring_initial_heading"] = data["heading"]
                            print(f"Modo vetoração (re)iniciado para {first_wp['name']}. Aguardando curva.")
                    else:
                        print("Nenhum waypoint gravado para navegar.")

                # Clique no botão ZERAR WAYPOINTS
                if gui.btn_clear_wps.collidepoint(mouse_pos):
                    if nav_state["recorded_waypoints"]:
                        nav_state["recorded_waypoints"] = []
                        nav_state["navigation_mode"] = "manual"
                        nav_state["active_waypoint_index"] = 0
                        nav_state["active"] = False
                        nav_state["vectoring_phase"] = 0
                        nav_state["circuit_side"] = None
                        nav_state["vnav_target_alt"] = None
                        nav_state["active_procedure"] = None
                        wp_counter = 0
                        course_text = ""
                        vnav_alt_text = ""
                        print("Todos os waypoints zerados. Contador reiniciado.")
                    else:
                        print("Nenhum waypoint para zerar.")
                # Clique no Dropdown aberto
                clicked_on_dropdown = False
                if show_chart_dropdown and hasattr(gui, 'dropdown_rects'):
                    for opt, rect in gui.dropdown_rects:
                        if rect.collidepoint(mouse_pos):
                            if opt == "NENHUMA (MANUAL)":
                                selected_chart_index = -1
                                selected_chart_name = None
                            else:
                                selected_chart_name = opt
                                selected_chart_index = available_chart_keys.index(opt)
                            show_chart_dropdown = False
                            clicked_on_dropdown = True
                            print(f"Carta Selecionada: {opt}")
                            break
                    if not clicked_on_dropdown:
                        # Clicou fora, fecha o dropdown
                        show_chart_dropdown = False
                        clicked_on_dropdown = True # Para não acionar outros botões embaixo
                
                if show_dir_dropdown and hasattr(gui, 'dir_dropdown_rects') and not clicked_on_dropdown:
                    for idx, rect in gui.dir_dropdown_rects:
                        if rect.collidepoint(mouse_pos):
                            wps = nav_state["recorded_waypoints"]
                            if wps and nav_state.get("active_procedure"):
                                print(f"*** DIRECT-TO MANUAL! Redirecionando para {wps[idx]['name']} ***")
                                nav_state["circuit_side"] = None
                                nav_state["vectoring_phase"] = 0
                                nav_state["active_waypoint_index"] = idx
                                target_wp = wps[idx]
                                nav_state["lat_orig"] = data["lat"]
                                nav_state["lon_orig"] = data["lon"]
                                bearing_true = NavMath.bearing_degrees(data["lat"], data["lon"], target_wp["lat"], target_wp["lon"])
                                mag_var = data.get("mag_var", 0.0)
                                bearing_mag = (bearing_true - mag_var + 360) % 360
                                nav_state["course_true"] = bearing_true
                                nav_state["course"] = bearing_mag
                                course_text = f"{int(bearing_mag):03}"
                                nav_state["vnav_target_alt"] = target_wp.get("alt")
                                vnav_alt_text = str(int(target_wp.get("alt", 0))) if target_wp.get("alt") else ""
                            show_dir_dropdown = False
                            clicked_on_dropdown = True
                            break
                    if not clicked_on_dropdown:
                        show_dir_dropdown = False
                        clicked_on_dropdown = True

                if not clicked_on_dropdown:
                    # Clique no botão DIR->
                    if hasattr(gui, 'btn_dir') and gui.btn_dir and gui.btn_dir.collidepoint(mouse_pos):
                        show_dir_dropdown = not show_dir_dropdown
                    
                    # Clique no botão TOGA
                    if hasattr(gui, 'btn_toga') and gui.btn_toga and gui.btn_toga.collidepoint(mouse_pos):
                        wps = nav_state["recorded_waypoints"]
                        if wps and nav_state.get("active_procedure"):
                            print("*** ARREMETIDA MANUAL (TOGA)! Redirecionando para MAHF ***")
                            nav_state["circuit_side"] = None
                            nav_state["vectoring_phase"] = 0
                            nav_state["active_waypoint_index"] = len(wps) - 1 # Pula pro último (MAHF)
                            target_wp = wps[-1]
                            nav_state["lat_orig"] = data["lat"]
                            nav_state["lon_orig"] = data["lon"]
                            bearing_true = NavMath.bearing_degrees(data["lat"], data["lon"], target_wp["lat"], target_wp["lon"])
                            mag_var = data.get("mag_var", 0.0)
                            bearing_mag = (bearing_true - mag_var + 360) % 360
                            nav_state["course_true"] = bearing_true
                            nav_state["course"] = bearing_mag
                            course_text = f"{int(bearing_mag):03}"
                            nav_state["vnav_target_alt"] = target_wp.get("alt")
                            vnav_alt_text = str(int(target_wp.get("alt", 0)))
                            # Força atualização imediata
                            continue

                    # Clique no botão CYCLE CHART (Abre o dropdown)
                    if hasattr(gui, 'btn_cycle_chart') and gui.btn_cycle_chart.collidepoint(mouse_pos):
                        show_chart_dropdown = not show_chart_dropdown

                    if hasattr(gui, 'btn_load_chart') and gui.btn_load_chart.collidepoint(mouse_pos) and selected_chart_name:
                        chart_data = AVAILABLE_ROUTES[selected_chart_name]
                        wps = []
                        mag_var = data.get("mag_var", 0.0)
                        
                        route_wps = chart_data["waypoints"]
                        mahf_lat = chart_data["mahf_lat"]
                        mahf_lon = chart_data["mahf_lon"]
                        
                        for i in range(len(route_wps)):
                            wp_info = route_wps[i]
                            # Se não for a pista (último), calcula rumo pro próximo fixo
                            if i < len(route_wps) - 1:
                                next_wp = route_wps[i+1]
                                brg_true = NavMath.bearing_degrees(wp_info["lat"], wp_info["lon"], next_wp["lat"], next_wp["lon"])
                            else:
                                # Se for a pista, o rumo é direto para o MAHF
                                brg_true = NavMath.bearing_degrees(wp_info["lat"], wp_info["lon"], mahf_lat, mahf_lon)
                                
                            brg_mag = (brg_true - mag_var + 360) % 360
                            
                            wps.append({
                                "name": wp_info["name"],
                                "lat": wp_info["lat"],
                                "lon": wp_info["lon"],
                                "alt": wp_info["alt_ft"],
                                "course": brg_mag,
                                "course_true": brg_true,
                                "desc": wp_info.get("desc", "")
                            })
                            
                        # Adiciona o MAHF no final. Seu curso de saída é irrelevante (fim da linha).
                        wps.append({
                            "name": chart_data["mahf_name"],
                            "lat": mahf_lat,
                            "lon": mahf_lon,
                            "alt": chart_data["mahf_alt"],
                            "course": wps[-1]["course"],
                            "course_true": wps[-1]["course_true"],
                            "desc": "MAHF"
                        })

                        nav_state["recorded_waypoints"] = wps
                        nav_state["navigation_mode"] = "manual" # Será 'route' quando clicar em NAVEGAR WAYPOINTS
                        nav_state["active_waypoint_index"] = 0
                        nav_state["active"] = False
                        nav_state["vectoring_phase"] = 0
                        nav_state["circuit_side"] = None
                        nav_state["vnav_target_alt"] = None
                        nav_state["active_procedure"] = selected_chart_name # Grava a carta ativa
                        wp_counter = len(wps)
                        print(f"Procedimento {selected_chart_name} carregado com sucesso! ({len(wps)} fixos incluindo MAHF). Clique em NAVEGAR WYP para iniciar.")



            elif event.type == pygame.KEYDOWN:
                # Trata entrada de teclado dependendo de qual campo está com foco ativo
                if active_input == "course":
                    if event.key == pygame.K_RETURN:
                        if course_text.isdigit():
                            course_val = int(course_text) % 360
                            if data.get("connected"):
                                # Só captura novas coordenadas de origem se nenhuma estiver definida (ex: 0.0)
                                # Caso contrário, mantém a origem original para calcular o desvio acumulado corretamente
                                if nav_state["lat_orig"] == 0.0 and nav_state["lon_orig"] == 0.0:
                                    nav_state["lat_orig"] = data["lat"]
                                    nav_state["lon_orig"] = data["lon"]
                                nav_state["course"] = float(course_val)
                                mag_var = data.get("mag_var", 0.0)
                                nav_state["course_true"] = (float(course_val) + mag_var) % 360
                                nav_state["active"] = True
                                print(f"Curso de Navegação Travado Manualmente: {course_val:03}° (True: {int(nav_state['course_true']):03}°)")
                            active_input = "none"
                    elif event.key == pygame.K_BACKSPACE:
                        course_text = course_text[:-1]
                    else:
                        # Curso manual aceita apenas números inteiros e no máximo 3 caracteres
                        if event.unicode.isdigit() and len(course_text) < 3:
                            course_text += event.unicode

                elif active_input == "tolerance":
                    if event.key == pygame.K_RETURN:
                        try:
                            tolerance_val = float(tolerance_text) if tolerance_text else 0.3
                            if tolerance_val <= 0:
                                tolerance_val = 0.3
                            nav_state["xtk_tolerance"] = tolerance_val
                            print(f"Tolerância XTK atualizada para: {tolerance_val} NM")
                        except ValueError:
                            print("Valor de tolerância inválido. Mantendo anterior.")
                        active_input = "none"
                    elif event.key == pygame.K_BACKSPACE:
                        tolerance_text = tolerance_text[:-1]
                    else:
                        # Tolerância aceita números e ponto decimal
                        if event.unicode.isdigit() or (event.unicode == '.' and '.' not in tolerance_text):
                            if len(tolerance_text) < 5:
                                tolerance_text += event.unicode

                elif active_input == "vnav_alt":
                    if event.key == pygame.K_RETURN:
                        try:
                            if vnav_alt_text:
                                nav_state["vnav_target_alt"] = float(vnav_alt_text)
                                print(f"Altitude Alvo VNAV atualizada para: {nav_state['vnav_target_alt']} FT")
                        except ValueError:
                            print("Altitude VNAV inválida.")
                        active_input = "none"
                    elif event.key == pygame.K_BACKSPACE:
                        vnav_alt_text = vnav_alt_text[:-1]
                    else:
                        if event.unicode.isdigit() and len(vnav_alt_text) < 5:
                            vnav_alt_text += event.unicode

                elif active_input == "vnav_tol":
                    if event.key == pygame.K_RETURN:
                        try:
                            tol_val = float(vnav_tol_text) if vnav_tol_text else 200.0
                            if tol_val < 0:
                                tol_val = 200.0
                            nav_state["vnav_tolerance"] = tol_val
                            print(f"Tolerância VNAV atualizada para: {tol_val} FT")
                        except ValueError:
                            print("Tolerância VNAV inválida.")
                        active_input = "none"
                    elif event.key == pygame.K_BACKSPACE:
                        vnav_tol_text = vnav_tol_text[:-1]
                    else:
                        if event.unicode.isdigit() and len(vnav_tol_text) < 5:
                            vnav_tol_text += event.unicode
                            
                elif active_input == "vnav_cal":
                    if event.key == pygame.K_RETURN:
                        try:
                            if vnav_cal_text:
                                desired_alt = float(vnav_cal_text)
                                raw_alt = source.get_data().get("alt", 0)
                                offset = desired_alt - raw_alt
                                nav_state["alt_offset"] = offset
                                nav_state["alt_calibrated"] = True
                                print(f"Altitude Calibrada! Desejada: {desired_alt} | Raw: {raw_alt} | Offset gerado: {offset}")
                        except ValueError:
                            print("Valor de calibração inválido.")
                        active_input = "none"
                    elif event.key == pygame.K_BACKSPACE:
                        vnav_cal_text = vnav_cal_text[:-1]
                    else:
                        # Permite sinal negativo e dígitos
                        if event.unicode in "-0123456789" and len(vnav_cal_text) < 6:
                            vnav_cal_text += event.unicode

                elif active_input == "gs_rwy_alt":
                    # Campo ALTITUDE RWY: aceita apenas dígitos inteiros (altitude do aeródromo)
                    if event.key == pygame.K_RETURN:
                        active_input = "none"
                    elif event.key == pygame.K_BACKSPACE:
                        gs_rwy_alt_text = gs_rwy_alt_text[:-1]
                    else:
                        if event.unicode.isdigit() and len(gs_rwy_alt_text) < 5:
                            gs_rwy_alt_text += event.unicode
                
                # Trata controles de vento lateral/deriva no modo MOCK
                elif mode_chosen == "mock":
                    if event.key == pygame.K_r:
                        source.drift_mode = "right"
                        print("Simulando ventos empurrando o avião para a DIREITA.")
                    elif event.key == pygame.K_l:
                        source.drift_mode = "left"
                        print("Simulando ventos empurrando o avião para a ESQUERDA.")
                    elif event.key == pygame.K_c:
                        source.drift_mode = "none"
                        print("Cancelando ventos laterais. Voo estabilizado.")
 
        # Atualiza a física de navegação se estiver ativa e houver dados válidos
        if nav_state["active"] and data.get("connected"):
            # ── Modo Rota: auto-sequenciamento de waypoints ──────────────────
            if nav_state["navigation_mode"] == "route":
                wps = nav_state["recorded_waypoints"]
                idx = nav_state["active_waypoint_index"]

                if idx < len(wps):
                    target_wp = wps[idx]
                    v_phase = nav_state.get("vectoring_phase", 0)
                    
                    if v_phase > 0:
                        # ── Nova Lógica Geométrica Contínua em Tempo Real (FMS Vectoring) ──
                        # Diferente de sistemas baseados em cronômetros, este algoritmo utiliza a
                        # geometria espacial contínua para guiar a aeronave em procedimentos (arremetidas, esperas).
                        current_hdg = data["heading"]
                        wp_lat = target_wp["lat"]
                        wp_lon = target_wp["lon"]
                        course_true = target_wp["course_true"]
                        
                        # 1. Cálculos de Posição Espacial (ATK e XTK)
                        # Distância reta (Haversine) até o waypoint
                        dist = NavMath.haversine_distance_nm(data["lat"], data["lon"], wp_lat, wp_lon)
                        
                        # Calcula a distância e a diferença de bearing do WP PARA o avião
                        bearing_from_wp = NavMath.bearing_degrees(wp_lat, wp_lon, data["lat"], data["lon"])
                        dist_from_wp = NavMath.haversine_distance_nm(wp_lat, wp_lon, data["lat"], data["lon"])
                        
                        # Signed ATK: distância projetada ao longo do curso 
                        # Positivo = à frente do WP01 (no sentido do curso), Negativo = atrás do WP01 (na aproximação)
                        angle_diff_rad = math.radians(bearing_from_wp - course_true)
                        atk_signed = dist_from_wp * math.cos(angle_diff_rad)
                        
                        xtk_nm = nav_state.get("xtk", 0.0)
                        
                        # 2. Identificação Dinâmica do Lado do Circuito (Direita/Esquerda)
                        # Em modo GS Sintético, calcula XTK relativo à cabeceira e curso INBOUND.
                        # Em modo normal (procedimento), usa XTK da rota.
                        circuit_side = nav_state.get("circuit_side")
                        if not circuit_side:
                            if nav_state.get("gs_synthetic_active") and not nav_state.get("active_procedure"):
                                # Calcula desvio em relação ao curso da pista (inbound)
                                xtk_inbound, _ = NavMath.calculate_xtk_by_course(
                                    data["lat"], data["lon"],
                                    nav_state["gs_thr_lat"], nav_state["gs_thr_lon"],
                                    nav_state["gs_inbound_course_true"]
                                )
                                # Se afastar mais de 0.5 NM para um dos lados, define o lado do circuito
                                if abs(xtk_inbound) > 0.5:
                                    circuit_side = "R" if xtk_inbound > 0 else "L"
                                    nav_state["circuit_side"] = circuit_side
                                    print(f"GS Sintético: Lado do circuito identificado como {'DIREITA' if circuit_side == 'R' else 'ESQUERDA'} (XTK={xtk_inbound:.2f} NM).")
                            else:
                                # Modo normal: usa XTK > 0.5 NM
                                if abs(xtk_nm) > 0.5:
                                    circuit_side = "R" if xtk_nm > 0 else "L"
                                    nav_state["circuit_side"] = circuit_side
                                    print(f"Geometria: Lado do circuito identificado automaticamente como {'DIREITA' if circuit_side == 'R' else 'ESQUERDA'}.")

                        # 3. Classificação Dinâmica da Perna Atual (State Machine real-time)
                        hdg_diff = (current_hdg - target_wp["course"] + 180) % 360 - 180
                        abs_hdg_diff = abs(hdg_diff)
                        
                        # Bearing direto até o WP (para usar como heading de interceptação)
                        bearing_to_wp = NavMath.bearing_degrees(data["lat"], data["lon"], wp_lat, wp_lon)
                        
                        if not circuit_side:
                            nav_state["vectoring_phase"] = 1  # Identificando lado
                        else:
                            # PRIORIDADE 1: Alinhado na final e após o WP (vindo de frás)
                            # Identifica por: proa correta + XTK pequeno
                            if abs(xtk_nm) <= 1.5 and abs_hdg_diff <= 45:
                                nav_state["vectoring_phase"] = 7  # Final / Inbound
                                    
                                nav_state["lat_orig"] = wp_lat
                                nav_state["lon_orig"] = wp_lon
                                nav_state["course_true"] = course_true
                                nav_state["course"] = target_wp["course"]
                                course_text = f"{int(target_wp['course']):03}"
                                # Salva bearing direto ao WP para guia de interceptação
                                nav_state["direct_bearing"] = bearing_to_wp
                            
                            elif abs_hdg_diff >= 120:
                                # Voando no rumo oposto (Afastamento)
                                if nav_state.get("gs_synthetic_active") and not nav_state.get("active_procedure"):
                                    # No modo GS Sintético de aproximação, curva para a base imediatamente no través
                                    if atk_signed <= 0.0:
                                        nav_state["vectoring_phase"] = 5  # Passou o través: Comanda Base Imediatamente!
                                    else:
                                        nav_state["vectoring_phase"] = 2
                                else:
                                    # Modo de Arremetida Padrão: afasta 3 NM adicionais após o través
                                    if atk_signed <= -3.0:
                                        nav_state["vectoring_phase"] = 4  # Distancia ideal atingida -> BASE
                                    elif atk_signed <= 0.0:
                                        nav_state["vectoring_phase"] = 3  # Passou o través do WP1
                                    else:
                                        nav_state["vectoring_phase"] = 2  # Perna Paralela (ainda antes do WP1)
                            
                            # PRIORIDADE 3: Voando de lado (Base ou Crosswind perpendicular)
                            elif 45 < abs_hdg_diff < 120:
                                is_turning_in = (circuit_side == "R" and hdg_diff < 0) or (circuit_side == "L" and hdg_diff > 0)
                                
                                if is_turning_in:
                                    # TRANSIÇÃO PARA FINAL: Se o XTK já está pequeno (≤ 2 NM) o aviao cruzou o eixo
                                    # ou está próximo o suficiente para mandar curvar para o inbound
                                    if abs(xtk_nm) <= 2.0:
                                        nav_state["vectoring_phase"] = 6  # Interceptação / INBOUND
                                        nav_state["direct_bearing"] = bearing_to_wp
                                    else:
                                        nav_state["vectoring_phase"] = 5  # Perna Base
                                else:
                                    nav_state["vectoring_phase"] = 1  # Crosswind inicial (afastando)
                            
                            # PRIORIDADE 4: Interceptação longa (proa intermediária)
                            else:
                                nav_state["vectoring_phase"] = 6  # Interceptação

                    # ── Detecção Automática de Arremetida (Missed Approach) ──
                    active_procedure = nav_state.get("active_procedure")
                    dist_to_wp = NavMath.haversine_distance_nm(
                        data["lat"], data["lon"],
                        target_wp["lat"], target_wp["lon"]
                    )
                    
                    if active_procedure and target_wp["name"].startswith("RWY") and dist_to_wp < 5.0:
                        chart_data = AVAILABLE_ROUTES.get(active_procedure)
                        if chart_data:
                            expected_turn = chart_data["missed_turn"]
                            hdg_diff_rwy = (data["heading"] - target_wp["course"] + 180) % 360 - 180
                            
                            is_turning_missed = False
                            if expected_turn == "R" and hdg_diff_rwy > 45:
                                is_turning_missed = True
                            elif expected_turn == "L" and hdg_diff_rwy < -45:
                                is_turning_missed = True
                                
                            if is_turning_missed:
                                print(f"*** ARREMETIDA DETECTADA! Curva >45° para {expected_turn} identificada. Redirecionando para MAHF ({chart_data['mahf_name']}) ***")
                                nav_state["circuit_side"] = None
                                nav_state["vectoring_phase"] = 0
                                nav_state["active_waypoint_index"] = len(wps) - 1
                                target_wp = wps[-1] # MAHF
                                nav_state["lat_orig"] = data["lat"]
                                nav_state["lon_orig"] = data["lon"]
                                bearing_true = NavMath.bearing_degrees(data["lat"], data["lon"], target_wp["lat"], target_wp["lon"])
                                mag_var = data.get("mag_var", 0.0)
                                bearing_mag = (bearing_true - mag_var + 360) % 360
                                nav_state["course_true"] = bearing_true
                                nav_state["course"] = bearing_mag
                                course_text = f"{int(bearing_mag):03}"
                                nav_state["vnav_target_alt"] = target_wp.get("alt")
                                vnav_alt_text = str(int(target_wp.get("alt", 0)))
                                continue # Pula o resto do processamento, recalcula na próxima iteração

                    # ── Rastreamento e Verificação de Captura de Waypoint (Global) ──
                    # Independente da fase de vetoração (mesmo se não cumpriu a curva),
                    # se cruzar o waypoint alvo (ATC Direct), avança automaticamente.
                    
                    # Verifica se passou a menos de 0.3 NM do waypoint (considerado atingido)
                    # OU se cruzou a linha de través (passou o waypoint) estando a uma distância razoável (< 3.0 NM)
                    # OU se a distância começou a aumentar (CPA - Closest Point of Approach) estando a < 3.0 NM
                    # OU Antecipação de Curva (Fly-By) se for um procedimento e estiver a <= 1.5 NM
                    
                    pass_std = nav_state.get("gs_synthetic_active") and not nav_state.get("active_procedure")
                    if pass_std:
                        # Modo GS Sintético: ignora o sequenciamento padrão baseado em CPA/Abeam
                        # Ele sequencia o alvo baseado estritamente na distância até a pista!
                        # Isso garante que no vetoramento ele segure o GS10, e na final faça o countdown correto.
                        dist_to_thr = nav_state.get("gs_dist_to_threshold")
                        v_phase = nav_state.get("vectoring_phase", 0)
                        
                        if dist_to_thr is not None:
                            if v_phase >= 6:  # Na interceptação ou final
                                # Calcula o WP à frente do avião. Se a distância é 9.7, o alvo é GS09 (index 1).
                                idx_calc = int(10 - math.floor(dist_to_thr))
                                if idx_calc < 0: idx_calc = 0
                                if idx_calc > len(wps) - 1: idx_calc = len(wps) - 1
                                
                                if idx_calc != idx:
                                    nav_state["active_waypoint_index"] = idx_calc
                                    target_wp = wps[idx_calc]
                                    nav_state["lat_orig"] = target_wp["lat"]
                                    nav_state["lon_orig"] = target_wp["lon"]
                                    print(f"GS Sintético: Sequenciado para {target_wp['name']} (Dist: {dist_to_thr:.1f} NM)")
                                    
                            # Se já chegou na cabeceira, encerra.
                            if dist_to_thr < 0.2 and idx == len(wps) - 1:
                                nav_state["navigation_mode"] = "route_finished"
                                nav_state["active"] = False
                                print("Chegada à cabeceira atingida.")
                            
                    if not pass_std:
                        is_mapt = target_wp["name"].startswith("RWY") or "MAPT" in target_wp.get("desc", "")
                        is_faf = target_wp.get("desc") == "FAF"
                    
                        if is_mapt:
                            WP_CAPTURE_RADIUS_NM = 0.15
                        elif is_faf:
                            WP_CAPTURE_RADIUS_NM = 0.1
                        else:
                            WP_CAPTURE_RADIUS_NM = 0.3
                    
                        bearing_to_wp = NavMath.bearing_degrees(data["lat"], data["lon"], target_wp["lat"], target_wp["lon"])
                        bearing_diff = (bearing_to_wp - nav_state["course_true"] + 180) % 360 - 180
                        passed_abeam = abs(bearing_diff) >= 90 and dist_to_wp < 3.0
                    
                        # Lógica CPA (Closest Point of Approach)
                        prev_dist = nav_state.get("prev_dist_to_wp")
                        passed_cpa = False
                        if prev_dist is not None:
                            # Se a distância aumentou em mais de 0.05 NM e estamos a menos de 3 NM do alvo
                            if dist_to_wp > prev_dist + 0.05 and dist_to_wp < 3.0:
                                passed_cpa = True
                        nav_state["prev_dist_to_wp"] = dist_to_wp
                    
                        active_procedure = nav_state.get("active_procedure")
                        # Antecipação de curva (Fly-By) dinâmico
                        turn_anticipation = False
                        if active_procedure and not is_mapt:
                            # Identifica o próximo fixo (faz looping para o IAF se estiver no MAHF)
                            next_idx = 0 if idx == len(wps) - 1 else idx + 1
                            next_wp_temp = wps[next_idx]
                        
                            # Calcula a diferença de proa entre a perna atual e a próxima perna
                            cur_course = nav_state.get("course", 0)
                            next_course = next_wp_temp.get("course", 0)
                            turn_angle = abs((next_course - cur_course + 180) % 360 - 180)
                        
                            # Define a distância de antecipação baseada no ângulo da curva
                            if target_wp.get("desc") == "FAF":
                                flyby_dist = 0.1  # Teste: FAF quase sem antecipação (Fly-Over suave)
                            elif turn_angle >= 100:
                                flyby_dist = 2.0  # Curvas super fechadas (ex: retorno do MAHF)
                            elif turn_angle >= 60:
                                flyby_dist = 1.2  # Curvas de 90 graus
                            else:
                                flyby_dist = 0.6  # Curvas suaves
                            
                            if dist_to_wp <= flyby_dist:
                                turn_anticipation = True
                    
                        if dist_to_wp < WP_CAPTURE_RADIUS_NM or passed_abeam or turn_anticipation or passed_cpa:
                            if dist_to_wp < WP_CAPTURE_RADIUS_NM:
                                reason = "distância"
                            elif turn_anticipation:
                                reason = "antecipação de curva (fly-by)"
                            elif passed_abeam:
                                reason = "passagem pelo través"
                            else:
                                reason = "afastamento (CPA)"
                            
                            nav_state["circuit_side"] = None
                            nav_state["prev_dist_to_wp"] = None
                            print(f"Waypoint {target_wp['name']} atingido (por {reason}, dist={dist_to_wp:.3f} NM). Avançando para próximo.")
                            nav_state["vectoring_phase"] = 0
                            next_idx = idx + 1
                            if next_idx < len(wps):
                                nav_state["active_waypoint_index"] = next_idx
                                next_wp = wps[next_idx]
                                active_procedure = nav_state.get("active_procedure")
                            
                                # ── LÓGICA DE ARREMETIDA ──────────────────────────────────
                                # Se o waypoint que acabamos de cruzar é o MAPT (pista),
                                # NÃO ativa o FD para o MAHF ainda. Entra em modo de subida
                                # monitorada até 4000 ft, conforme carta SBSJ.
                                if is_mapt and active_procedure:
                                    print("*** MAPT cruzado. Iniciando subida de ARREMETIDA. FD suspenso até 4000 ft. ***")
                                    nav_state["missed_approach_climbing"] = True
                                    nav_state["active"] = False          # Suspende FD lateral
                                    nav_state["vnav_gs_alt"] = None      # Suspende GS
                                    nav_state["vnav_target_alt"] = None  # Suspende alerta de altitude
                                    vnav_alt_text = ""
                                else:
                                    # Waypoint normal: avança normalmente
                                    if active_procedure:
                                        nav_state["lat_orig"] = target_wp["lat"]
                                        nav_state["lon_orig"] = target_wp["lon"]
                                        nav_state["course_true"] = target_wp["course_true"]
                                        nav_state["course"] = target_wp["course"]
                                        bearing_mag = target_wp["course"]
                                        course_text = f"{int(bearing_mag):03}"
                                    else:
                                        nav_state["lat_orig"] = data["lat"]
                                        nav_state["lon_orig"] = data["lon"]
                                        bearing_true = NavMath.bearing_degrees(data["lat"], data["lon"], next_wp["lat"], next_wp["lon"])
                                        mag_var = data.get("mag_var", 0.0)
                                        bearing_mag = (bearing_true - mag_var + 360) % 360
                                        nav_state["course_true"] = bearing_true
                                        nav_state["course"] = bearing_mag
                                        course_text = f"{int(bearing_mag):03}"
                                
                                    nav_state["vnav_target_alt"] = next_wp.get("alt")
                                    vnav_alt_text = str(int(next_wp.get("alt", 0))) if next_wp.get("alt") else ""
                                    print(f"Novo alvo: {next_wp['name']} | Vetoração Curso: {int(bearing_mag):03}°")
                            else:
                                active_procedure = nav_state.get("active_procedure")
                                if active_procedure:
                                    print("MAHF atingido. Loop de procedimento! Retornando ao IAF para nova aproximação.")
                                    nav_state["active_waypoint_index"] = 0
                                    next_wp = wps[0]
                                    nav_state["lat_orig"] = data["lat"]
                                    nav_state["lon_orig"] = data["lon"]
                                    bearing_true = NavMath.bearing_degrees(data["lat"], data["lon"], next_wp["lat"], next_wp["lon"])
                                    mag_var = data.get("mag_var", 0.0)
                                    bearing_mag = (bearing_true - mag_var + 360) % 360
                                    nav_state["course_true"] = bearing_true
                                    nav_state["course"] = bearing_mag
                                    course_text = f"{int(bearing_mag):03}"
                                    nav_state["vnav_target_alt"] = next_wp.get("alt")
                                    vnav_alt_text = str(int(next_wp.get("alt", 0))) if next_wp.get("alt") else ""
                                    print(f"Novo alvo (Loop): {next_wp['name']} | Curso: {int(bearing_mag):03}°")
                                else:
                                    # Último waypoint atingido: encerra a rota
                                    print("Último waypoint atingido. Rota concluída.")
                                    nav_state["vectoring_phase"] = 0
                                    nav_state["navigation_mode"] = "route_finished"
                                    nav_state["active"] = False
                                    nav_state["vnav_target_alt"] = None
                                    vnav_alt_text = ""

                # ── Cálculo do XTK e Proa de Intercepção (comum a ambos os modos) ─
            v_phase_current = nav_state.get("vectoring_phase", 0)
            
            # Calcula o Cross-Track Error (XTK) continuamente para todas as fases
            xtk_nm, dist = NavMath.calculate_xtk_by_course(
                data["lat"], data["lon"],
                nav_state["lat_orig"], nav_state["lon_orig"],
                nav_state["course_true"]
            )
            nav_state["xtk"] = xtk_nm
            
            # --- CÁLCULO DE GLIDEPATH (VNAV GS) ---
            nav_state["vnav_gs_alt"] = None
            if nav_state.get("active") and nav_state.get("active_procedure"):
                idx = nav_state.get("active_waypoint_index", 0)
                wps = nav_state.get("recorded_waypoints", [])
                
                # Se estamos na perna final (alvo é MAPT/RWY e viemos do FAF)
                if 0 < idx < len(wps):
                    target_wp = wps[idx]
                    # Só ativa a rampa de 3 graus (GS) se estivermos na perna final (alvo atual é o MAPT)
                    if target_wp["name"].startswith("RWY") or "MAPT" in target_wp.get("desc", ""):
                        proc_name = nav_state["active_procedure"]
                        if proc_name in AVAILABLE_ROUTES:
                            rwy_elev = AVAILABLE_ROUTES[proc_name].get("rwy_elev", 2120)
                            tch = 50 # Threshold Crossing Height padrão (50 pés)
                            
                            dist_to_mapt = NavMath.haversine_distance_nm(
                                data["lat"], data["lon"],
                                target_wp["lat"], target_wp["lon"]
                            )
                            
                            if dist_to_mapt > 0:
                                # Rampa geométrica perfeita LPV/WAAS de 3.0 graus (318.4 pés por milha náutica)
                                # Ancorada exatamente sobre a cabeceira da pista (Elevação + TCH de 50 pés)
                                vnav_gs_alt = (rwy_elev + tch) + (dist_to_mapt * 318.4)
                                nav_state["vnav_gs_alt"] = vnav_gs_alt

            # --- CÁLCULO DE GLIDEPATH SINTÉTICO (GS Sintético: rampa de 3° sem procedimento formal) ---
            # Ativo sempre que gs_synthetic_active=True, independente de active_procedure.
            # Calcula a distância até a cabeceira salva e a altitude alvo na rampa.
            if nav_state.get("gs_synthetic_active") and not nav_state.get("active_procedure"):
                thr_lat = nav_state.get("gs_thr_lat")
                thr_lon = nav_state.get("gs_thr_lon")
                rwy_elev = nav_state.get("gs_rwy_elev", 0)
                if thr_lat is not None and thr_lon is not None:
                    dist_to_thr = NavMath.haversine_distance_nm(
                        data["lat"], data["lon"], thr_lat, thr_lon
                    )
                    nav_state["gs_dist_to_threshold"] = dist_to_thr
                    # Rampa de 3° ancorada nos 50 FT acima da pista
                    # Só ativa o GS (barra VNAV) quando dentro de 10 NM
                    if dist_to_thr <= 10.0 and dist_to_thr > 0:
                        gs_alt = (rwy_elev + 50) + (dist_to_thr * 318.4)
                        nav_state["vnav_gs_alt"] = gs_alt

            if v_phase_current == 0 or v_phase_current == 7:
                # Fase normal ou INBOUND: calcula heading de interceptação via XTK em tempo real
                max_ang = 15.0 if nav_state.get("active_procedure") else 45.0
                target_heading_true = NavMath.calculate_intercept_heading(
                    nav_state["course_true"], xtk_nm, max_intercept=max_ang
                )
                mag_var = data.get("mag_var", 0.0)
                nav_state["target_heading"] = int((target_heading_true - mag_var + 360) % 360)
            
            elif v_phase_current == 6:
                # INTERCEPTAÇÃO: usa bearing direto ao WP para máxima segurança
                direct_bearing = nav_state.get("direct_bearing")
                if direct_bearing is not None:
                    mag_var = data.get("mag_var", 0.0)
                    nav_state["target_heading"] = int((direct_bearing - mag_var + 360) % 360)
                else:
                    max_ang = 15.0 if nav_state.get("active_procedure") else 45.0
                    target_heading_true = NavMath.calculate_intercept_heading(
                        nav_state["course_true"], xtk_nm, max_intercept=max_ang
                    )
                    mag_var = data.get("mag_var", 0.0)
                    nav_state["target_heading"] = int((target_heading_true - mag_var + 360) % 360)
            
            elif v_phase_current == 2 or v_phase_current == 3 or v_phase_current == 4:
                # PERNA DE AFASTAMENTO / PARALELA: proa perfeitamente recíproca à pista
                # Fixa exatos 180 graus em relação à pista para afastar em linha reta perfeita
                course_true_val = nav_state.get("course_true", nav_state["course"])
                reciprocal_true = (course_true_val + 180) % 360
                mag_var = data.get("mag_var", 0.0)
                nav_state["target_heading"] = int((reciprocal_true - mag_var + 360) % 360)
            
            elif v_phase_current == 5:
                # PERNA BASE: Fixa exatos 90 graus em relação à pista, sem correção de XTK!
                course_true_val = nav_state.get("course_true", nav_state["course"])
                c_side = nav_state.get("circuit_side")
                if c_side == "L":
                    base_true = (course_true_val + 90) % 360
                else:
                    base_true = (course_true_val - 90 + 360) % 360
                mag_var = data.get("mag_var", 0.0)
                nav_state["target_heading"] = int((base_true - mag_var + 360) % 360)
            
            elif v_phase_current == 1:
                # CROSSWIND / IDENTIFICANDO: heading fixo (ainda sem XTK significativo)
                c_side = nav_state.get("circuit_side")
                course_mag = nav_state["course"]
                if c_side:
                    nav_state["target_heading"] = int((course_mag + 180) % 360)
                else:
                    nav_state["target_heading"] = int(course_mag)
            
            else:
                nav_state["target_heading"] = int(nav_state["course"])
        else:
            nav_state["xtk"] = 0.0
            nav_state["target_heading"] = 0

        # --- MONITORAMENTO DE SUBIDA NA ARREMETIDA ---
        # Este bloco é executado SEMPRE que o sistema está ligado (system_on),
        # independente do FD estar ativo ou não. Assim funciona mesmo com active=False.
        if nav_state.get("missed_approach_climbing") and nav_state.get("active_procedure") and data.get("connected"):
            ma_target = nav_state.get("missed_approach_target_alt", 4000)
            current_alt = data.get("alt", 0)
            nav_state["vnav_target_alt"] = None  # Limpa alerta normal de altitude

            if current_alt >= ma_target:
                print(f"*** {int(ma_target)} ft atingidos ({current_alt:.0f} ft). Ativando FD para MAHF. ***")
                nav_state["missed_approach_climbing"] = False

                # Aponta o FD direto para o MAHF a partir da posição atual
                proc_name = nav_state["active_procedure"]
                wps = nav_state.get("recorded_waypoints", [])
                mahf_wp = wps[-1] if wps else None

                if mahf_wp:
                    mag_var = data.get("mag_var", 0.0)
                    bearing_true = NavMath.bearing_degrees(data["lat"], data["lon"], mahf_wp["lat"], mahf_wp["lon"])
                    bearing_mag = (bearing_true - mag_var + 360) % 360
                    nav_state["lat_orig"] = data["lat"]
                    nav_state["lon_orig"] = data["lon"]
                    nav_state["course_true"] = bearing_true
                    nav_state["course"] = bearing_mag
                    course_text = f"{int(bearing_mag):03}"
                    nav_state["active_waypoint_index"] = len(wps) - 1
                    nav_state["navigation_mode"] = "route"
                    nav_state["vnav_target_alt"] = mahf_wp.get("alt")
                    vnav_alt_text = str(int(mahf_wp.get("alt", 0))) if mahf_wp.get("alt") else ""
                    nav_state["active"] = True
                    nav_state["vectoring_phase"] = 0
                    print(f"FD ativado! Direto para MAHF: {mahf_wp['name']} | Curso: {int(bearing_mag):03}°")

        # RENDERIZAÇÃO
        canvas.fill(BLACK)

        if data.get("connected"):
            # Desenha o Horizonte Artificial na Esquerda (Pitch e Bank já corrigidos)
            gui.draw_horizon(canvas, data["pitch"], data["bank"])

            # Desenha as barras do Flight Director sobre o horizonte (se ativo e configurado)
            gui.draw_flight_director(canvas, nav_state, data.get("alt", 0))

            # Desenha a miniatura do avião por cima do Flight Director
            gui.draw_aircraft_reference(canvas)

            # Desenha a escala vertical de waypoints no canto esquerdo
            gui.draw_waypoint_scale(
                canvas,
                nav_state["recorded_waypoints"],
                nav_state["active_waypoint_index"],
                nav_state["navigation_mode"]
            )

            # Desenha as Caixas de Telemetria Básica
            # HDG (Proa Magnética)
            gui.draw_data_box(canvas, HDG_RECT, "HDG", f"{int(data['heading']):03}°")
            # TRK (Rumo Real)
            gui.draw_data_box(canvas, TRK_RECT, "TRK", f"{int(data['track']):03}°", color=YELLOW)
            # SPD (Velocidade Indicada)
            gui.draw_data_box(canvas, SPD_RECT, "SPD", f"{int(data['speed'])}")
            # ALT (Altitude)
            gui.draw_data_box(canvas, ALT_RECT, "ALT", f"{int(data['alt'])}")

            # Desenha o Painel de Navegação Lateral na Direita (com suporte a foco duplo de input)
            gui.draw_navigation_panel(canvas, data, nav_state, mouse_pos, active_input, course_text, tolerance_text, gs_rwy_alt_text)
            
            # Desenha o Painel de Navegação Vertical na Direita Extra
            gui.draw_vnav_panel(canvas, data, nav_state, mouse_pos, active_input, vnav_alt_text, vnav_tol_text, vnav_cal_text)
            # Desenha o Painel de Procedimento (Cartas) na Direita Extra Inferior
            gui.dropdown_rects, gui.dir_dropdown_rects = gui.draw_procedure_panel(
                canvas, mouse_pos, selected_chart_name, available_chart_keys, show_chart_dropdown, nav_state, show_dir_dropdown
            )
        else:
            # Sem conexão com simulador/sensores
            if data.get("waiting_for_flight"):
                gui.draw_text(canvas, "Conectado. Aguardando voo carregar...", (1340 // 2, 900 // 2), gui.f_title, YELLOW, align="center")
            else:
                gui.draw_text(canvas, "Sem conexao com o simulador / sensores", (1340 // 2, 900 // 2), gui.f_title, (255, 50, 50), align="center")

        # Escala e desenha na janela física
        draw_scaled_canvas(screen, canvas, window_size)
        pygame.display.flip()
        clock.tick(30)

    # Limpeza final
    source.disconnect()
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()


