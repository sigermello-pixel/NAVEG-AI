"""
================================================================================
NAVEG-AI - SISTEMA COMPLETO DE NAVEGACAO E DIRETOR DE VOO (PFD / HUD)
ARQUIVO UNICO CONSOLIDADO DE PRODUCAO (STANDALONE FULL PRODUCTION)
Data de Homologacao de Voo: 31/10/2026
Autor / Engenharia: Equipe NAVEG-AI
Compatibilidade: Raspberry Pi 4/5 (Raspberry Pi OS Bookworm) e Windows PC (MSFS 2020)

MODULOS INTEGRADOS NO ARQUIVO CONSOLIDADO:
  - 1. Importacoes globais, encoding UTF-8 e dependencias de sistema operacional
  - 2. nav_math.py (Calculos geodesicos de navegacao esferica, Cross-Track Error e vetoracao)
  - 3. routes.py (Cartas e rotas RNAV/RNP, fixos IAF/IF/FAF/MAPT/MAHF e aproximacoes)
  - 4. data_sources.py (Fontes SimConnect MSFS, Sensores Raspberry Pi BNO055/GPS e Mock)
  - 5. naveg_ai_copilot.py (Co-Piloto Neural com IA Local Qwen 2.5 via Ollama HTTP API)
  - 6. pfd_gui.py (Interface visual PFD/HUD, Horizonte Artificial, Glide Slope e Telas Touch)
  - 7. monitor_lateral.py (Logica de controle de voo, VNAV, GS 50FT, histerese e loop 30 FPS)
================================================================================
"""

# ============================================================================
# 1. IMPORTACOES GLOBAIS E CONFIGURACOES DE SISTEMA OPERACIONAL
# ============================================================================
import sys

# Garante que a saida padrao do terminal (stdout) suporte caracteres UTF-8 (acentos e graus)
if sys.stdout and hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

# Garante que a saida de erros (stderr) suporte caracteres UTF-8
if sys.stderr and hasattr(sys.stderr, 'reconfigure'):
    try:
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

# Bibliotecas de manipulacao do sistema de arquivos e comandos do sistema operacional
import os          # Operacoes de caminho de arquivo, variaveis de ambiente e checagem de disco
import glob        # Busca de arquivos de log com padrao de nomenclatura (wildcards)
import argparse    # Processamento de argumentos de linha de comando (--mode pi/sim/mock/udp)

# Biblioteca grafica para renderizacao da avionica e interface PFD/HUD
import pygame      # Renderizacao grafica acelerada, desenho de vetores e captura de eventos

# Bibliotecas matematicas e temporais
import time        # Marcadores de timestamp de telemetria e calculo de delta tempo (dt)
import math        # Funcoes trigonometricas para trigonometria esferica e projecoes de voo
import random      # Geracao de perturbacoes e ruidos atmosfericos no modo de teste simulado (Mock)
import datetime    # Formatacao de data/hora ISO para gravacao dos arquivos de log de voo (.csv)

# Bibliotecas de comunicacao, serializacao de dados e processamento paralelo
import json        # Serializacao de telemetria e leitura do arquivo sensor_calibration.json
import threading   # Execucao em segundo plano do leitor GPS e das inferencias da rede neural
import urllib.request # Cliente HTTP nativo para envio de prompts ao servidor local do Ollama
import re          # Expressoes regulares para parsing robusto das respostas geradas pelo Qwen 2.5
import socket      # Comunicacao via sockets de rede UDP para recepcao remota de telemetria

# ============================================================================
# 2. MATEMATICA DE NAVEGACAO E GEODESIA ESFERICA (nav_math.py)
# ============================================================================
class NavMath:
    """
    Classe utilitaria estatica contendo formulas matematicas geodesicas e de navegacao aerea.
    Implementa calculos de grande circulo (Great Circle), determinacao de Cross-Track Error (XTK),
    geometria de interceptacao de curso, projecao direta de waypoints e rampa sintetica Glide Slope.
    """
    # Constante oficial do Raio Medio da Terra em Milhas Nauticas (WGS-84 aproximado)
    EARTH_RADIUS_NM = 3440.065  # 1 Milha Nautica = 1852 metros (Raio terrestre = 6371 km)

    @staticmethod
    def haversine_distance_nm(lat1, lon1, lat2, lon2):
        """
        Calcula a distancia de grande circulo em Milhas Nauticas (NM) entre duas coordenadas.
        Utiliza a Formula de Haversine para estabilidade numerica em distancias curtas e longas.
        
        Parametros:
          lat1, lon1: Latitude e Longitude do ponto de origem em graus decimais.
          lat2, lon2: Latitude e Longitude do ponto de destino em graus decimais.
        
        Retorno:
          Distancia geodesica entre os pontos expressa em Milhas Nauticas (NM).
        """
        # Converte as coordenadas de graus decimais para radianos
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        # Calcula as diferencas angulares entre as latitudes e longitudes
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        # Aplica a formula trigonometrica de Haversine
        a = (math.sin(dlat / 2) ** 2 + 
             math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2)
        
        # Garante que o valor dentro da raiz quadrada permaneca no intervalo valido [0.0, 1.0]
        c = 2 * math.asin(math.sqrt(max(0.0, min(1.0, a))))
        
        # Multiplica a distancia angular pelo raio da Terra para obter Milhas Nauticas
        return NavMath.EARTH_RADIUS_NM * c

    @staticmethod
    def bearing_degrees(lat1, lon1, lat2, lon2):
        """
        Calcula o rumo inicial verdadeiro (Initial Bearing / Forward Azimuth) de (lat1, lon1) para (lat2, lon2).
        Utiliza a relacao trigonometrica esferica atan2(y, x).
        
        Parametros:
          lat1, lon1: Coordenadas do ponto inicial em graus decimais.
          lat2, lon2: Coordenadas do ponto final em graus decimais.
          
        Retorno:
          Rumo em graus no circulo de 0 a 359.99 graus em relacao ao Norte Verdadeiro.
        """
        # Converte as coordenadas de entrada de graus para radianos
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        
        # Componentes do vetor esferico
        y = math.sin(dlon) * math.cos(lat2)
        x = (math.cos(lat1) * math.sin(lat2) - 
             math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
        
        # Calcula o angulo azimutal com arco tangente de dois argumentos
        bearing = math.atan2(y, x)
        
        # Converte para graus e normaliza dentro do circulo compasso de 0 a 360 graus
        return (math.degrees(bearing) + 360) % 360

    @staticmethod
    def normalize_angle(angle):
        """
        Normaliza qualquer angulo angular para o intervalo simetrico de -180 a +180 graus.
        Fundamental para computar desvios de proa sem ambiguidades de transicao pelo Norte (360/0).
        """
        return (angle + 180) % 360 - 180

    @staticmethod
    def calculate_xtk_by_course(lat_curr, lon_curr, lat_orig, lon_orig, course_deg):
        """
        Calcula o Cross-Track Error (XTK) em Milhas Nauticas (NM) em relacao a um curso dinamico.
        Utiliza a projecao esferica do triangulo de navegacao aerea.
        
        Parametros:
          lat_curr, lon_curr: Posicao geografica atual da aeronave.
          lat_orig, lon_orig: Ponto de referencia do fixo ou cabeceira da rota.
          course_deg: Rumo desejado (inbound course) em graus.
        
        Retorna:
          xtk_nm: Positivo se a aeronave estiver a DIREITA do curso, Negativo se estiver a ESQUERDA.
          dist_from_orig: Distancia direta em NM da aeronave ate o ponto de origem.
        """
        # Distancia e bearing do ponto de origem ate a posicao atual da aeronave
        dist_from_orig = NavMath.haversine_distance_nm(lat_orig, lon_orig, lat_curr, lon_curr)
        
        # Evita indeterminacao matematica caso a aeronave esteja exatamente sobre a origem
        if dist_from_orig < 0.0001:
            return 0.0, 0.0

        # Rumo verdadeiro da origem para a aeronave
        bearing_from_orig = NavMath.bearing_degrees(lat_orig, lon_orig, lat_curr, lon_curr)

        # Diferenca angular entre o rumo percorrido desde a origem e o curso desejado
        angle_diff = NavMath.normalize_angle(bearing_from_orig - course_deg)
        angle_diff_rad = math.radians(angle_diff)

        # Formula esferica exata para Cross-Track Distance (distancia ortogonal a rota)
        sin_xt = math.sin(dist_from_orig / NavMath.EARTH_RADIUS_NM) * math.sin(angle_diff_rad)
        sin_xt = max(-1.0, min(1.0, sin_xt)) # Previne estouro numerico de ponto flutuante
        xtk_nm = math.asin(sin_xt) * NavMath.EARTH_RADIUS_NM

        return xtk_nm, dist_from_orig

    @staticmethod
    def calculate_intercept_heading(course_deg, xtk_nm, sensitivity=80.0, max_intercept=25.0, deadband_nm=0.010, dist_to_wp_nm=None):
        """
        Calcula a proa sugerida pelo algoritmo de interceptacao para retornar ao eixo do curso.
        Implementa regras de navegacao IFR e afunilamento de seguranca:
        - Quando dist_to_wp_nm <= 2.0 NM: afunila o corte de 25 graus para 8 graus perto da cabeceira.
        - Fora de 2.0 NM: corte assertivo ate 25 graus (minimo 8 graus fora do eixo).
        - Zona de rollout nos ultimos ~130m (0.07 NM) para alinhamento suave e sem overshoot.
        """
        # Se o desvio lateral estiver dentro da zona morta de precisao (deadband), mantem o curso
        if abs(xtk_nm) <= deadband_nm:
            return int(round(course_deg)) % 360

        # Afunilamento do angulo maximo de corte na aproximacao final para a pista
        if dist_to_wp_nm is not None and dist_to_wp_nm <= 2.0:
            ratio = max(0.0, min(1.0, dist_to_wp_nm / 2.0))
            max_allowed = 8.0 + (max_intercept - 8.0) * ratio
        else:
            max_allowed = max_intercept

        # Distancia do limiar de captura suave (0.07 NM = ~130 metros)
        capture_dist = 0.07
        if abs(xtk_nm) < capture_dist:
            # Zona de Captura Antecipada (elimina o overshoot de interceptacao)
            # Afunila suavemente nos ultimos 130m do eixo de voo
            ramp = max(0.0, (abs(xtk_nm) - deadband_nm) / (capture_dist - deadband_nm))
            base_cut = min(max_allowed, 10.0)
            intercept_angle = base_cut * ramp
        else:
            # Interceptacao assertiva com ganho proporcional
            intercept_angle = min(max_allowed, max(8.0, abs(xtk_nm) * sensitivity))

        # Aplica a correcao para o lado oposto ao desvio
        if xtk_nm > 0:
            # Aeronave a direita da rota -> vira para a esquerda (subtrai angulo)
            target_heading = (course_deg - intercept_angle + 360.0) % 360.0
        else:
            # Aeronave a esquerda da rota -> vira para a direita (adiciona angulo)
            target_heading = (course_deg + intercept_angle) % 360.0

        return int(round(target_heading)) % 360

    @staticmethod
    def calculate_course_deviation_advice(runway_course, current_course, xtk_nm=0.0, was_correcting=False,
                                          deadband_deg=2.5, trigger_deg=5.0,
                                          max_correction_deg=25.0, xtk_gain=80.0, lead_gain=0.5,
                                          dist_to_runway_nm=None, dist_to_wp_nm=None):
        """
        MODO LOG / LNAV: sugere em tempo real o curso (track) que leva a aeronave de volta ao EIXO.
        - Quando dist_to_runway_nm <= 2.0 NM: afunila dinamicamente de 25 graus ate 8 graus na cabeceira.
        - Acima de 2.0 NM: corte assertivo de 10 a 25 graus.
        - Rollout suave nos ultimos ~130m (0.07 NM) do eixo.
        - Corredor de estabilizacao no eixo central: 0.020 NM (~37 metros).
        """
        # Reduz dinamicamente o angulo maximo de interceptacao perto da pista (< 2.0 NM)
        if dist_to_runway_nm is not None:
            if dist_to_runway_nm <= 2.0:
                ratio = max(0.0, min(1.0, dist_to_runway_nm / 2.0))
                dynamic_max = 8.0 + (max_correction_deg - 8.0) * ratio
                max_correction_deg = min(max_correction_deg, dynamic_max)

        # Aplica afunilamento caso haja referencia de distancia para o proximo waypoint
        if dist_to_wp_nm is not None and dist_to_wp_nm <= 2.0:
            ratio = max(0.0, min(1.0, dist_to_wp_nm / 2.0))
            max_correction_deg = min(max_correction_deg, 8.0 + (max_correction_deg - 8.0) * ratio)

        # Desvio angular entre a trajetoria percorrida (ground track) e o eixo da pista
        deviation = NavMath.normalize_angle(current_course - runway_course)

        # Angulo de interceptacao (+ = alvo a direita do curso da pista), limitado ao maximo
        if abs(xtk_nm) <= 0.020:
            offset = 0.0
        elif abs(xtk_nm) < 0.07:
            # Zona de Captura Antecipada (< 0.07 NM): afunila o corte proporcionalmente ate zero no corredor
            ramp = max(0.0, (abs(xtk_nm) - 0.020) / (0.07 - 0.020))
            offset_mag = min(max_correction_deg, 10.0) * ramp
            offset = -math.copysign(offset_mag, xtk_nm)
        else:
            # Fora da zona de captura: corte proporcional robusto
            offset_mag = min(max_correction_deg, max(8.0, abs(xtk_nm) * xtk_gain))
            offset = -math.copysign(offset_mag, xtk_nm)

        # Determina o curso magnetico alvo somando o offset de correcao
        target = (runway_course + offset) % 360

        # Erro entre o curso atual e o curso sugerido (+ = curso atual a direita do alvo)
        error = NavMath.normalize_angle(current_course - target)
        abs_err = abs(error)

        # Logica de histerese para evitar oscilacoes continuas de instrucao ao piloto
        if abs_err >= trigger_deg:
            correcting = True
        elif abs_err <= deadband_deg:
            correcting = False
        else:
            correcting = was_correcting

        # Define o sentido da curva recomendada ("L" = Esquerda, "R" = Direita)
        turn = None
        if correcting:
            turn = "L" if error > 0 else "R"

        return {"deviation": deviation, "correcting": correcting, "turn": turn,
                "target_course": int(round(target)) % 360,
                "intercepting": abs(offset) >= 0.5 or abs(xtk_nm) >= 0.015}

    @staticmethod
    def project_point(lat_deg, lon_deg, bearing_deg, dist_nm):
        """
        Projeta um novo ponto geografico a 'dist_nm' Milhas Nauticas de (lat_deg, lon_deg) no rumo 'bearing_deg'.
        Usa a formula do Problema Direto da geodesia esferica baseada nas relacoes de Vincenty / Great Circle.
        
        Parametros:
          lat_deg, lon_deg: Coordenadas de origem em graus decimais.
          bearing_deg: Azimute / rumo de projecao em graus.
          dist_nm: Distancia a ser projetada em Milhas Nauticas.
          
        Retorna:
          Tupla (lat_nova, lon_nova) com as coordenadas projetadas em graus decimais.
        """
        # Raio da Terra em Milhas Nauticas
        R = NavMath.EARTH_RADIUS_NM
        
        # Converte os parametros de entrada para radianos
        lat1 = math.radians(lat_deg)
        lon1 = math.radians(lon_deg)
        brg  = math.radians(bearing_deg)
        d_R  = dist_nm / R # Distancia angular percorrida sobre a esfera

        # Calcula a nova latitude
        lat2 = math.asin(
            math.sin(lat1) * math.cos(d_R) +
            math.cos(lat1) * math.sin(d_R) * math.cos(brg)
        )
        
        # Calcula a nova longitude
        lon2 = lon1 + math.atan2(
            math.sin(brg) * math.sin(d_R) * math.cos(lat1),
            math.cos(d_R) - math.sin(lat1) * math.sin(lat2)
        )
        
        # Retorna as coordenadas convertidas de volta para graus decimais
        return math.degrees(lat2), math.degrees(lon2)

    @staticmethod
    def generate_gs_waypoints(rwy_thr_lat, rwy_thr_lon, inbound_course_true,
                              inbound_course_mag, rwy_elev_ft,
                              num_points=10, max_dist_nm=10.0,
                              gs_deg=3.0, tch_ft=50):
        """
        Gera N waypoints para a rampa sintetica de Glide Slope de 3 graus (ou gs_deg).
        
        A rampa e ancorada nos 50 pes (tch_ft - Threshold Crossing Height) acima da elevacao da pista,
        exatamente sobre a cabeceira (touchdown zone). Os waypoints sao projetados para TRAS da cabeceira
        (rumo reciproco do inbound_course_true) em intervalos regulares de 1 NM ate max_dist_nm.
        
        Parametros:
          rwy_thr_lat/lon: Coordenadas da cabeceira da pista em graus decimais.
          inbound_course_true: Curso VERDADEIRO de aproximacao final (inbound track).
          inbound_course_mag: Curso MAGNETICO de aproximacao.
          rwy_elev_ft: Elevacao oficial da pista em pes (MSL).
          num_points: Quantidade de waypoints gerados ao longo da rampa (padrao: 10).
          max_dist_nm: Distancia maxima da rampa de descida em NM (padrao: 10.0 NM).
          gs_deg: Inclinacao angular da rampa em graus (padrao ICAO: 3.0 graus).
          tch_ft: Altura de cruzamento da cabeceira em pes (padrao ICAO: 50 ft).
          
        Retorna:
          Lista de dicionarios com {name, lat, lon, alt, course, course_true, desc} ordenados
          do fixo mais distante (ex: GS10) em direcao a cabeceira da pista (GS01).
        """
        # Razao de descida vertical em pes por Milha Nautica:
        # tan(3 graus) * 6076.12 ft/NM ≈ 318.4 ft por Milha Nautica
        ft_per_nm = math.tan(math.radians(gs_deg)) * 6076.12

        # Rumo de afastamento da cabeceira (reciproco = curso + 180 graus)
        outbound_true = (inbound_course_true + 180) % 360
        outbound_mag  = (inbound_course_mag  + 180) % 360

        waypoints = []
        # Itera sobre cada ponto da rampa de descida
        for i in range(num_points):
            dist_nm = i * 1.0  # Incremento de 1 NM a cada ponto
            # Altitude do perfil = elevacao da pista + 50ft TCH + projecao trigonometrica da rampa
            alt_ft  = rwy_elev_ft + tch_ft + dist_nm * ft_per_nm

            # Projeta as coordenadas geograficas do ponto ao longo do rumo reciproco
            lat, lon = NavMath.project_point(
                rwy_thr_lat, rwy_thr_lon, outbound_true, dist_nm
            )

            # Adiciona o waypoint na lista temporaria
            waypoints.append({
                "name"        : f"GS{num_points - i:02d}",
                "lat"         : lat,
                "lon"         : lon,
                "alt"         : round(alt_ft, 1),
                "course"      : outbound_mag,
                "course_true" : outbound_true,
                "desc"        : "GS_SYNTH" if i > 0 else "GS_SYNTH (PISTA)",
            })

        # Os waypoints foram gerados da cabeceira para fora (0 a 9 NM).
        # Para que a navegacao ocorra de fora para dentro da rampa, invertemos a lista
        waypoints.reverse()
        # Ajusta os nomes sequenciais e o curso inbound (apontando para a pista)
        for idx, wp in enumerate(waypoints):
            wp["course"]       = inbound_course_mag
            wp["course_true"]  = inbound_course_true
            wp["name"]         = f"GS{idx+1:02d}"

        return waypoints


# ============================================================================
# 3. ROTAS RNAV / RNP E CARTAS DE PROCEDIMENTO AERONAUTICO (routes.py)
# ============================================================================
# Banco de dados de navegacao baseado nas Cartas Oficiais RNP do DECEA / AISWEB:
# Aerodromo: SBSJ - Sao Jose dos Campos / SP (Prof. Urbano Ernesto Stumpf)
# Pista 16: Coordenadas da cabeceira S23°13'11" / W045°52'19" | Elevacao: 2032 ft MSL
# Pista 34: Coordenadas da cabeceira S23°14'12" / W045°51'12" | Elevacao: 2121 ft MSL

# Procedimento RNP RWY 16 - Transicao via Fixo SJ063 (Setor Noroeste)
SBSJ_RNP_16_SJ063_ROUTE = [
    # IAF (Initial Approach Fix) - Ponto de inicio da aproximacao por instrumentos
    {"name": "SJ063", "lat": -22.995180, "lon": -46.115061, "alt_ft": 8000, "desc": "IAF"},
    # IF (Intermediate Fix) - Ponto de transicao e desaceleracao para configuracao da aeronave
    {"name": "SJ043", "lat": -23.030636, "lon": -46.076713, "alt_ft": 7000, "desc": "IF transicao"},
    # IF (Intermediate Fix) - Fixo intermediario de convergencia das trajetorias
    {"name": "GEMSO", "lat": -23.101513, "lon": -45.999958, "alt_ft": 5100, "desc": "IF"},
    # FAF (Final Approach Fix) - Ponto de inicio da descida final estabilizada para a pista
    {"name": "ANRAN", "lat": -23.160600, "lon": -45.935991, "alt_ft": 3680, "desc": "FAF"},
    # MAPT (Missed Approach Point) - Ponto de aproximacao perdida / Cabeceira da Pista 16
    {"name": "RWY16", "lat": -23.219722, "lon": -45.871944, "alt_ft": 2423, "desc": "MAPT (Pista 16)"}
]

# Procedimento RNP RWY 16 - Transicao via Fixo LONES (Setor Norte)
SBSJ_RNP_16_LONES_ROUTE = [
    # IAF (Initial Approach Fix) - Entrada da aproximacao vindo do setor LONES
    {"name": "LONES", "lat": -23.030516, "lon": -45.923333, "alt_ft": 7000, "desc": "IAF"},
    # IF (Intermediate Fix) - Interceptacao do segmento comum de aproximacao
    {"name": "GEMSO", "lat": -23.101513, "lon": -45.999958, "alt_ft": 5100, "desc": "IF"},
    # FAF (Final Approach Fix) - Início da rampa de descida final de 3 graus
    {"name": "ANRAN", "lat": -23.160600, "lon": -45.935991, "alt_ft": 3680, "desc": "FAF"},
    # MAPT (Missed Approach Point) - Cruzamento da cabeceira da Pista 16
    {"name": "RWY16", "lat": -23.219722, "lon": -45.871944, "alt_ft": 2423, "desc": "MAPT (Pista 16)"}
]

# Procedimento RNP RWY 34 - Transicao via Fixo DAGOV (Setor Sul / Sudeste)
SBSJ_RNP_34_DAGOV_ROUTE = [
    # IAF (Initial Approach Fix) - Ponto inicial no setor DAGOV
    {"name": "DAGOV", "lat": -23.437333, "lon": -45.635305, "alt_ft": 7000, "desc": "IAF"},
    # IF (Intermediate Fix) - Alinhamento com a perna intermediaria
    {"name": "SJ046", "lat": -23.354752, "lon": -45.725200, "alt_ft": 5300, "desc": "IF"},
    # FAF (Final Approach Fix) - Fixacao do glide path final
    {"name": "SJ047", "lat": -23.295752, "lon": -45.789352, "alt_ft": 3770, "desc": "FAF"},
    # MAPT (Missed Approach Point) - Cabeceira da Pista 34
    {"name": "RWY34", "lat": -23.236667, "lon": -45.853333, "alt_ft": 2571, "desc": "MAPT (Pista 34)"}
]

# Procedimento RNP RWY 34 - Transicao via Fixo SOTPI (Setor Leste)
SBSJ_RNP_34_SOTPI_ROUTE = [
    # IAF (Initial Approach Fix) - Fixo de entrada do procedimento
    {"name": "SOTPI", "lat": -23.245444, "lon": -45.626411, "alt_ft": 7000, "desc": "IAF"},
    # IF (Intermediate Fix) - Transicao para a perna intermediaria
    {"name": "SJ046", "lat": -23.354752, "lon": -45.725200, "alt_ft": 5300, "desc": "IF"},
    # FAF (Final Approach Fix) - Fixo de descida final estabilizada
    {"name": "SJ047", "lat": -23.295752, "lon": -45.789352, "alt_ft": 3770, "desc": "FAF"},
    # MAPT (Missed Approach Point) - Ponto de arremetida / Cabeceira 34
    {"name": "RWY34", "lat": -23.236667, "lon": -45.853333, "alt_ft": 2571, "desc": "MAPT (Pista 34)"}
]

# Dicionario mestre de rotas disponiveis no computador de voo (FMC / FMS)
# Permite selecionar qualquer carta de aproximacao diretamente na interface grafica touch
AVAILABLE_ROUTES = {
    "SBSJ 16 (SJ063)": {
        "waypoints": SBSJ_RNP_16_SJ063_ROUTE, # Sequencia de fixos de navegacao
        "rwy_elev": 2032,                    # Elevacao da cabeceira RWY16 em pes MSL (AIP Brasil)
        "mahf_name": "LONES",                # Fixo de espera de aproximacao perdida (Missed Approach)
        "mahf_lat": -23.030516,              # Latitude do fixo de espera
        "mahf_lon": -45.923333,              # Longitude do fixo de espera
        "mahf_alt": 7000,                    # Altitude minima no fixo de espera
        "missed_turn": "L"                   # Curva a esquerda em caso de arremetida
    },
    "SBSJ 16 (LONES)": {
        "waypoints": SBSJ_RNP_16_LONES_ROUTE,
        "rwy_elev": 2032,
        "mahf_name": "LONES",
        "mahf_lat": -23.030516,
        "mahf_lon": -45.923333,
        "mahf_alt": 7000,
        "missed_turn": "L"
    },
    "SBSJ 34 (DAGOV)": {
        "waypoints": SBSJ_RNP_34_DAGOV_ROUTE,
        "rwy_elev": 2121,                    # Elevacao da cabeceira RWY34 em pes MSL (AIP Brasil)
        "mahf_name": "SOTPI",
        "mahf_lat": -23.245444,
        "mahf_lon": -45.626411,
        "mahf_alt": 7000,
        "missed_turn": "R"                   # Curva a direita em caso de arremetida
    },
    "SBSJ 34 (SOTPI)": {
        "waypoints": SBSJ_RNP_34_SOTPI_ROUTE,
        "rwy_elev": 2121,
        "mahf_name": "SOTPI",
        "mahf_lat": -23.245444,
        "mahf_lon": -45.626411,
        "mahf_alt": 7000,
        "missed_turn": "R"
    }
}

# 4. FONTES DE DADOS E SENSORES (data_sources.py)
# ============================================================================
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
    """
    Classe base abstrata para provedores de telemetria da aeronave (Hardware Pi, SimConnect, UDP e Mock).
    Padroniza o carregamento de calibracao de atitude (sensor_calibration.json), deteccao de atualizacoes de arquivo
    em tempo real e aplicacao de offsets angulares de arfagem (pitch), inclinacao lateral (bank) e proa (heading).
    """
    def __init__(self):
        """Inicializa a classe base de telemetria, configurando flags de conexao e carregando a calibracao dos sensores."""
        # Flag que indica se a fonte de dados esta conectada e operacional
        self.connected = False
        # Dicionario de calibracao de atitude com offsets de pitch, bank e heading
        self.calibration = {}
        # Timestamp mtime da ultima atualizacao do arquivo sensor_calibration.json
        self.cal_mtime = 0
        # Temporizador para verificacao periodica de alteracoes no arquivo de calibracao
        self.last_cal_check = 0
        self.last_true_pitch = 0.0
        self.last_true_bank = 0.0
        # Executa a carga inicial dos fatores de calibracao salvos em disco
        self.load_calibration()

    def load_calibration(self):
        """Carrega os fatores de calibracao (offsets angulares e flags de inversao) do arquivo sensor_calibration.json."""
        cal_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sensor_calibration.json")
        if not os.path.exists(cal_path):
            cal_path = "sensor_calibration.json"
        if os.path.exists(cal_path):
            try:
                self.cal_mtime = os.path.getmtime(cal_path)
                with open(cal_path, "r", encoding="utf-8") as f:
                    self.calibration = json.load(f)
            except Exception:
        # Dicionario de calibracao de atitude com offsets de pitch, bank e heading
                self.calibration = {}
        else:
        # Dicionario de calibracao de atitude com offsets de pitch, bank e heading
            self.calibration = {}

    def check_calibration_update(self):
        """Verifica periodicamente (a cada 0.5s) se o arquivo de calibracao foi modificado em disco para recarregamento a quente."""
        now = time.time()
        if now - self.last_cal_check > 0.5:
            self.last_cal_check = now
            cal_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sensor_calibration.json")
            if not os.path.exists(cal_path):
                cal_path = "sensor_calibration.json"
            if os.path.exists(cal_path):
                try:
                    mtime = os.path.getmtime(cal_path)
                    if mtime > self.cal_mtime:
        # Executa a carga inicial dos fatores de calibracao salvos em disco
                        self.load_calibration()
                except Exception:
                    pass

    def apply_attitude_calibration(self, raw_pitch, raw_bank, raw_heading):
        """Aplica as correcoes de offset e inversao de eixos aos dados brutos de arfagem, rolamento e proa."""
        p_offset = self.calibration.get("pitch_offset", 0.0)
        b_offset = self.calibration.get("bank_offset", 0.0)
        h_offset = self.calibration.get("heading_offset", 0.0)
        inv_pitch = -1.0 if self.calibration.get("invert_pitch", False) else 1.0
        inv_bank = -1.0 if self.calibration.get("invert_bank", False) else 1.0
        hdg_180 = 180.0 if self.calibration.get("invert_heading", False) else 0.0

        cal_p = (raw_pitch - p_offset) * inv_pitch
        cal_b = (raw_bank - b_offset) * inv_bank
        cal_h = (raw_heading + hdg_180 + h_offset) % 360.0
        return cal_p, cal_b, cal_h

    def connect(self):
        """Metodo abstrato para estabelecer conexao com a fonte de telemetria fisica ou virtual."""
        raise NotImplementedError

    def get_data(self):
        """Metodo abstrato que retorna o dicionario padronizado com os parametros de telemetria da aeronave."""
        raise NotImplementedError

    def disconnect(self):
        """Metodo abstrato para encerrar a conexao e liberar portas seriais e descritores de sockets."""
        pass


class SimConnectSource(TelemetrySource):
    """
    Provedor de telemetria integrado ao Microsoft Flight Simulator (MSFS 2020 / 2024) via API SimConnect.
    Coleta variaveis de voo em tempo real (altitude indicada, velocidade no ar IAS, coordenadas GPS, atitude espacial,
    variacao magnetica e razao de subida/descida VSI) com tratamento automatico de reconexao.
    """
    def __init__(self):
        """Inicializa o cliente SimConnect para comunicacao com o Microsoft Flight Simulator."""
        super().__init__()
        self.sm = None
        self.aq = None
        self.last_alt = None
        self.last_time = None
        self.current_vsi = 0.0

    def connect(self):
        """Estabelece sessao com o servidor SimConnect local do Flight Simulator."""
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
            
            # Bank (rolar) em radianos: invertemos o sinal para alinhar o horizonte do PFD perfeitamente com o MSFS
            bank_rad = self.aq.get("PLANE_BANK_DEGREES") or 0.0
            bank = -math.degrees(bank_rad)
            
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
                
            # Coleta de Velocidade Vertical (VSI em ft/min)
            vsi_raw = self.aq.get("VERTICAL_SPEED")
            now = time.time()
            if self.last_alt is not None and self.last_time is not None and now > self.last_time:
                dt = now - self.last_time
                if dt >= 0.03:
                    calc_vsi = ((alt - self.last_alt) / dt) * 60.0
                    self.current_vsi = self.current_vsi * 0.6 + calc_vsi * 0.4
                    self.last_alt = alt
                    self.last_time = now
            else:
                self.last_alt = alt
                self.last_time = now
                self.current_vsi = 0.0

            if vsi_raw is not None:
                # SimConnect AircraftRequests já retorna VERTICAL_SPEED em feet/minute
                vsi = float(vsi_raw)
            else:
                vsi = self.current_vsi

            # Angle of Attack (AoA) em radianos -> graus
            aoa_rad = self.aq.get("INCIDENCE_ALPHA") or 0.0
            aoa = math.degrees(aoa_rad)

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
                "bank": bank,
                "aoa": aoa,
                "vsi": vsi,
                "satellites": 12,
                "gps_quality": 1,
                "gps_fix": True,
                "is_real_pi": False
            }
        except Exception as e:
            print(f"Erro ao ler telemetria do SimConnect: {e}")
            self.connected = False
            return {"connected": False}

    def disconnect(self):
        """Encerra a sessao SimConnect com o simulador e redefine os estados internos."""
        if self.sm:
            self.sm.exit()
            print("SimConnect desconectado.")
        self.connected = False


class PiSensorSource(TelemetrySource):
    """
    Provedor de telemetria de voo real para Raspberry Pi 4 / 5 integrado a sensores fisicos de avionica.
    Le coordenadas e velocidade de solo (Ground Speed) do GPS u-blox Neo-M8N via UART em thread paralela de alta taxa,
    e atitude espacial (Pitch, Bank, Heading) do sensor inercial 9-DOF Bosch BNO055 via barramento I2C.
    """
    def __init__(self, gps_port=None, gps_baud=9600):
        """Inicializa os drivers de hardware para o GPS Neo-M8N (UART) e sensor IMU BNO055 (I2C) na Raspberry Pi."""
        super().__init__()
        self.gps_port = gps_port
        self.gps_baud = gps_baud
        self.ser = None
        self.i2c = None
        self.imu = None
        self.calibration = {}
        
        # Últimos dados salvos para caso de falha de leitura rápida
        self.last_lat = -23.6277   # Default SBSP
        self.last_lon = -46.6546
        self.last_speed = 0.0
        self.last_track = 0.0
        self.last_alt = 2600.0
        self.last_true_pitch = 0.0
        self.last_true_bank = 0.0
        self.last_pitch = 0.0
        self.last_bank = 0.0
        self.last_heading = None
        self.satellites = 0
        self.gps_quality = 0
        self.last_gps_time = 0.0
        self._bno_addr = 0x29
        self._imu_err_count = 0

        # --- Ancoragem da proa no GPS (heading aiding) ---
        # O BNO055 perde a calibracao do magnetometro a cada boot (mag=0). Sem ele a proa so
        # gira por giroscopio e DERIVA (medido: ~+6 graus/min). O rumo do GPS (COG) e usado
        # como ancora, somente em reta e com velocidade suficiente.
        self.HDG_AID_MIN_SPEED = 10.0     # kt: abaixo disso o COG do GPS nao e confiavel
        self.HDG_AID_DEADBAND = 12.0      # graus: preserva o angulo de deriva (crab) do aviao
        self.HDG_AID_TAU = 5.0            # s: constante de tempo da correcao
        self.HDG_AID_MAX_RATE = 3.0       # graus/s: limite de taxa (sem saltos visiveis)
        self.HDG_AID_MAX_TURN = 4.0       # graus/s: em curva o COG atrasa, nao corrige
        self._aid_offset = 0.0
        self._aid_last_t = 0.0
        self._aid_last_hdg = None
        self._aid_last_trk = None

        # --- Resolucao do TRK (rumo) sem salto de fonte ---
        self.TRK_GPS_ENTER_SPEED = 5.0    # kt: passa a confiar no COG do GPS
        self.TRK_GPS_EXIT_SPEED = 3.5     # kt: deixa de confiar (histerese)
        self.TRK_GPS_DEBOUNCE = 1.0       # s acima de ENTER antes de confiar
        self.TRK_HOLD_TAU = 12.0          # s: no solo o TRK converge suavemente para a proa
        self.TRK_MAX_AGE = 2.0            # s: COG mais velho que isso nao e confiavel
        self.TRK_MAX_SLEW = 35.0          # graus/s: slew rate limiter (elimina saltos instantaneos)
        self._trk_gps_active = False
        self._trk_fast_since = 0.0
        self._trk_ref_off = 0.0
        self._trk_ref_t = 0.0
        self._trk_update_t = 0.0
        self._displayed_trk = None
        self._displayed_trk_t = 0.0

        self.last_cal_check = 0
        self.cal_mtime = 0
        
        # Thread de leitura contínua assíncrona do GPS (Zero bloqueio do Pygame)
        self._thread_running = False
        self._gps_thread = None
        self._data_lock = threading.Lock()

        # Carrega calibração se existir
        self.load_calibration()

    def load_calibration(self):
        """Carrega os offsets de alinhamento fisico dos sensores instalados na fuselagem da aeronave."""
        cal_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sensor_calibration.json")
        if not os.path.exists(cal_path):
            cal_path = "sensor_calibration.json"
        
        if os.path.exists(cal_path):
            try:
                self.cal_mtime = os.path.getmtime(cal_path)
                with open(cal_path, "r", encoding="utf-8") as f:
                    self.calibration = json.load(f)
                print(f"[CALIBRAÇÃO] Carregada com sucesso: {self.calibration}")
            except Exception as e:
                print(f"[CALIBRAÇÃO] Erro ao carregar '{cal_path}': {e}")
                self.calibration = {}
        else:
            print("[CALIBRAÇÃO] 'sensor_calibration.json' não encontrado. Usando calibração padrão (offsets = 0).")
            self.calibration = {}

    def _send_ubx(self, msg_class, msg_id, payload):
        """Envia pacote binário UBX para o GPS u-blox Neo-M8N com cálculo automático de checksum."""
        if not self.ser:
            return
        try:
            length = len(payload)
            header = bytes([0xB5, 0x62, msg_class, msg_id, length & 0xFF, (length >> 8) & 0xFF])
            body = header[2:] + payload
            ck_a = 0
            ck_b = 0
            for b in body:
                ck_a = (ck_a + b) & 0xFF
                ck_b = (ck_b + ck_a) & 0xFF
            packet = header + payload + bytes([ck_a, ck_b])
            self.ser.write(packet)
            self.ser.flush()
        except Exception:
            pass

    def configure_gps_high_rate(self):
        """Configura o GPS u-blox Neo-M8N para 5 Hz (200ms) mantendo sentenças RMC, GGA e VTG ativas."""
        if not self.ser:
            return
        try:
            print("[GPS] Otimizando Neo-M8N: ativando taxa de 5 Hz (200ms) e priorizando telemetria...")
            # 1. Desabilita GSV, GSA, GLL via UBX-CFG-MSG (0x06, 0x01) para poupar banda serial (mantém VTG habilitado)
            for msg_id in [0x03, 0x02, 0x01]:  # 0x03=GSV, 0x02=GSA, 0x01=GLL
                self._send_ubx(0x06, 0x01, bytes([0xF0, msg_id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
                time.sleep(0.02)

            # Comandos PUBX NMEA compatíveis universalmente com módulos u-blox
            for nmea_cmd in [
                b"$PUBX,40,GSV,0,0,0,0*59\r\n",
                b"$PUBX,40,GSA,0,0,0,0*4E\r\n",
                b"$PUBX,40,GLL,0,0,0,0*5C\r\n",
                b"$PUBX,40,VTG,0,1,0,0*5F\r\n",
                b"$PUBX,40,RMC,0,1,0,0*47\r\n",
                b"$PUBX,40,GGA,0,1,0,0*5A\r\n"
            ]:
                try:
                    self.ser.write(nmea_cmd)
                    time.sleep(0.02)
                except Exception:
                    pass
            self.ser.flush()

            # 2. Configura taxa de navegação para 5 Hz (200 ms) via UBX-CFG-RATE (0x06, 0x08)
            # measRate = 200 ms (0xC8, 0x00), navRate = 1, timeRef = 1 (GPS time)
            self._send_ubx(0x06, 0x08, bytes([0xC8, 0x00, 0x01, 0x00, 0x01, 0x00]))
            time.sleep(0.05)
            print("[GPS] [OK] Neo-M8N operando com RMC, GGA e VTG ativos a 5 Hz!")
        except Exception as e:
            print(f"[GPS] Aviso na configuração de taxa do GPS: {e}")

    def _apply_heading_aid(self, heading, spd, trk, now):
        """
        Ancora a proa do BNO055 no rumo do GPS para eliminar a deriva do giroscopio.
        - So atua em linha reta (proa e rumo estaveis) e acima de HDG_AID_MIN_SPEED.
        - Zona morta de HDG_AID_DEADBAND graus: o angulo de deriva (crab) do aviao contra
          vento de traves e preservado; apenas o excedente e corrigido.
        - Correcao suave e com taxa limitada: nunca provoca salto visivel na proa.
        Qualquer falha aqui devolve a proa original (nunca derruba a leitura da IMU).
        """
        try:
            if not getattr(self, "_trk_initialized", False):
                return heading
            if (now - getattr(self, "_trk_update_t", 0.0)) > self.TRK_MAX_AGE:
                return heading   # COG velho (GPS parado/sem fix): nunca ancora nele
            h_noaid = (heading - self._aid_offset) % 360.0
            if self._aid_last_t == 0.0 or self._aid_last_hdg is None:
                self._aid_last_t, self._aid_last_hdg, self._aid_last_trk = now, h_noaid, trk
                return heading
            dt = now - self._aid_last_t
            if dt < 0.2:
                return heading
            hdg_rate = abs((h_noaid - self._aid_last_hdg + 180.0) % 360.0 - 180.0) / dt
            trk_rate = abs((trk - self._aid_last_trk + 180.0) % 360.0 - 180.0) / dt
            self._aid_last_t, self._aid_last_hdg, self._aid_last_trk = now, h_noaid, trk
            if dt > 1.5 or spd < self.HDG_AID_MIN_SPEED:
                return heading
            if hdg_rate > self.HDG_AID_MAX_TURN or trk_rate > self.HDG_AID_MAX_TURN:
                return heading
            err = (trk - heading + 180.0) % 360.0 - 180.0
            if abs(err) <= self.HDG_AID_DEADBAND:
                return heading
            excess = err - math.copysign(self.HDG_AID_DEADBAND, err)
            step = excess * (1.0 - math.exp(-dt / self.HDG_AID_TAU))
            lim = self.HDG_AID_MAX_RATE * dt
            step = max(-lim, min(lim, step))
            self._aid_offset = (self._aid_offset + step) % 360.0
            return (heading + step) % 360.0
        except Exception:
            return heading

    def _resolve_track(self, heading, spd, trk, now):
        """
        Define o TRK exibido sem trocar bruscamente de fonte.
        - Em movimento (histerese 3.5/5.0 kt + 1 s de debounce): usa o rumo do GPS com COG fresco.
        - Devagar/parado: mantem o ultimo desvio (rumo GPS - proa) e o faz decair
          suavemente para a proa (regra de solo original), em vez de saltar para ela.
        - Slew rate limiter a 35°/s: elimina qualquer degrau visual instantâneo na tela.
        """
        try:
            if not getattr(self, "_trk_initialized", False):
                return heading
            fresh = (now - getattr(self, "_trk_update_t", 0.0)) <= self.TRK_MAX_AGE
            if spd >= self.TRK_GPS_ENTER_SPEED:
                if self._trk_fast_since == 0.0:
                    self._trk_fast_since = now
            elif spd < self.TRK_GPS_EXIT_SPEED:
                self._trk_fast_since = 0.0
                self._trk_gps_active = False

            if (self._trk_fast_since != 0.0 and not self._trk_gps_active and fresh
                    and (now - self._trk_fast_since) >= self.TRK_GPS_DEBOUNCE):
                self._trk_gps_active = True

            if self._trk_gps_active and fresh:
                self._trk_ref_off = (trk - heading + 180.0) % 360.0 - 180.0
                self._trk_ref_t = now
                target = trk
            elif self._trk_ref_t > 0.0:
                off = self._trk_ref_off * math.exp(-(now - self._trk_ref_t) / self.TRK_HOLD_TAU)
                target = (heading + off) % 360.0
            else:
                target = heading

            # Slew rate limiter: elimina degraus visuais instantâneos na tela (máximo 35°/s)
            if self._displayed_trk is None or self._displayed_trk_t == 0.0:
                self._displayed_trk = target
                self._displayed_trk_t = now
                return target

            dt = max(0.001, min(0.5, now - self._displayed_trk_t))
            self._displayed_trk_t = now
            diff = (target - self._displayed_trk + 180.0) % 360.0 - 180.0
            max_step = self.TRK_MAX_SLEW * dt
            if abs(diff) <= max_step:
                self._displayed_trk = target
            else:
                self._displayed_trk = (self._displayed_trk + math.copysign(max_step, diff)) % 360.0
            return self._displayed_trk
        except Exception:
            return trk if spd >= 3.5 else heading

    def _gps_reader_loop(self):
        """Thread daemon de alta performance: esvazia a porta serial continuamente sem travar o Pygame."""
        while self._thread_running and self.ser:
            try:
                # Esvazia backlog acumulado se houver atraso excessivo na fila serial (>2048 bytes)
                if hasattr(self.ser, 'in_waiting') and self.ser.in_waiting > 2048:
                    self.ser.reset_input_buffer()

                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue
                # Remove quaisquer bytes de ruído antes do início '$' da sentença
                dollar_idx = line.find('$')
                if dollar_idx != -1:
                    line = line[dollar_idx:]

                if line.startswith(('$GPRMC', '$GNRMC', '$GPGGA', '$GNGGA', '$GPVTG', '$GNVTG')):
                    msg = pynmea2.parse(line)
                    st = getattr(msg, 'sentence_type', '')
                    with self._data_lock:
                        if st == 'RMC':
                            # Padrão NMEA: status 'A' (Active) indica navegação válida.
                            # Não depende de pynmea2.is_valid pois ele rejeita NMEA 4.10 do u-blox.
                            status_val = getattr(msg, 'status', '')
                            is_valid_rmc = (status_val == 'A') or (hasattr(msg, 'is_valid') and msg.is_valid)
                            if is_valid_rmc:
                                lat = getattr(msg, 'latitude', None)
                                lon = getattr(msg, 'longitude', None)
                                if lat not in (None, "", 0.0) and lon not in (None, "", 0.0):
                                    try:
                                        self.last_lat = float(lat)
                                        self.last_lon = float(lon)
                                        self.last_gps_time = time.time()
                                    except (ValueError, TypeError):
                                        pass
                                spd = getattr(msg, 'spd_over_grnd', None)
                                if spd not in (None, ""):
                                    try:
                                        raw_spd = float(spd)
                                        prev_spd = getattr(self, "_raw_gps_spd", 0.0)
                                        self._raw_gps_spd = prev_spd * 0.45 + raw_spd * 0.55
                                        if raw_spd < 3.0 or self._raw_gps_spd < 2.8:
                                            self.last_speed = 0.0
                                        else:
                                            self.last_speed = self._raw_gps_spd
                                    except (ValueError, TypeError):
                                        pass
                                crs = getattr(msg, 'true_course', None)
                                if crs not in (None, "") and self.last_speed >= 3.5:
                                    try:
                                        new_crs = float(crs) % 360.0
                                        if getattr(self, "_trk_initialized", False):
                                            diff_c = (new_crs - self.last_track + 180.0) % 360.0 - 180.0
                                            self.last_track = (self.last_track + diff_c * 0.45) % 360.0
                                        else:
                                            self.last_track = new_crs
                                            self._trk_initialized = True
                                        self._trk_update_t = time.time()
                                    except (ValueError, TypeError):
                                        pass
                        elif st == 'GGA':
                            num_sats = getattr(msg, 'num_sats', None)
                            if num_sats not in (None, ""):
                                try:
                                    self.satellites = int(num_sats)
                                except (ValueError, TypeError):
                                    pass
                            qual = getattr(msg, 'gps_qual', None)
                            if qual not in (None, ""):
                                try:
                                    self.gps_quality = int(qual)
                                except (ValueError, TypeError):
                                    pass
                            is_valid_gga = (self.gps_quality > 0) or (hasattr(msg, 'is_valid') and msg.is_valid)
                            if is_valid_gga:
                                alt = getattr(msg, 'altitude', None)
                                if alt not in (None, ""):
                                    try:
                                        raw_alt_ft = float(alt) * 3.28084
                                        if self.last_speed < 3.0:
                                            # Parado: trava pequenos ruidos (< 4 ft) para tela nao tremer no solo
                                            if not getattr(self, "_alt_initialized", False) or abs(raw_alt_ft - self.last_alt) >= 4.0:
                                                self.last_alt = raw_alt_ft
                                                self._alt_initialized = True
                                        else:
                                            # Em movimento/voo: 100% DIRETO como era originalmente (zero atraso na subida)
                                            self.last_alt = raw_alt_ft
                                            self._alt_initialized = True
                                    except (ValueError, TypeError):
                                        pass
                                lat = getattr(msg, 'latitude', None)
                                lon = getattr(msg, 'longitude', None)
                                if lat not in (None, "", 0.0) and lon not in (None, "", 0.0):
                                    try:
                                        self.last_lat = float(lat)
                                        self.last_lon = float(lon)
                                        self.last_gps_time = time.time()
                                    except (ValueError, TypeError):
                                        pass
                        elif st == 'VTG':
                            # Sentença VTG: velocidade dedicada sobre o solo em nós e km/h
                            spd_kts = getattr(msg, 'spd_over_grnd_kts', None)
                            if spd_kts not in (None, ""):
                                try:
                                    raw_spd = float(spd_kts)
                                    prev_spd = getattr(self, "_raw_gps_spd", 0.0)
                                    self._raw_gps_spd = prev_spd * 0.45 + raw_spd * 0.55
                                    if raw_spd < 3.0 or self._raw_gps_spd < 2.8:
                                        self.last_speed = 0.0
                                    else:
                                        self.last_speed = self._raw_gps_spd
                                except (ValueError, TypeError):
                                    pass
                            trk = getattr(msg, 'true_track', None)
                            if trk not in (None, "") and self.last_speed >= 3.5:
                                try:
                                    new_trk = float(trk) % 360.0
                                    if getattr(self, "_trk_initialized", False):
                                        diff_t = (new_trk - self.last_track + 180.0) % 360.0 - 180.0
                                        self.last_track = (self.last_track + diff_t * 0.45) % 360.0
                                    else:
                                        self.last_track = new_trk
                                        self._trk_initialized = True
                                    self._trk_update_t = time.time()
                                except (ValueError, TypeError):
                                    pass
            except Exception:
                time.sleep(0.01)

    def connect(self):
        """Abre a porta serial do GPS, configura o barramento I2C do BNO055 e lanca a thread de leitura do GPS."""
        if not PI_SENSORS_AVAILABLE:
            print("Bibliotecas de sensores da Raspberry Pi (serial, pynmea2, adafruit-circuitpython-bno055) não estão disponíveis.")
            self.connected = False
            return False

        # Lista de portas candidatas na Raspberry Pi 5 / 4
        candidatos_porta = [self.gps_port] if self.gps_port else []
        candidatos_porta.extend(["/dev/ttyAMA0", "/dev/serial0", "/dev/ttyUSB0", "/dev/ttyACM0"])
        # Remove duplicados mantendo a ordem
        portas_para_tentar = []
        for p in candidatos_porta:
            if p and p not in portas_para_tentar:
                portas_para_tentar.append(p)

        # Conectar GPS via Porta Serial
        for port in portas_para_tentar:
            try:
                print(f"Tentando conectar ao GPS Neo M8N na porta {port}...")
                self.ser = serial.Serial(port, self.gps_baud, timeout=0.1)
                self.gps_port = port
                print(f"[OK] GPS Inicializado com sucesso em {port}!")
                break
            except Exception:
                continue

        if not self.ser:
            print("Aviso: Não foi possível abrir nenhuma porta serial do GPS. Tentando rodar com GPS simulado...")

        # Conectar IMU BNO055 via I2C
        try:
            print("Conectando ao sensor IMU BNO055 via I2C...")
            self.i2c = busio.I2C(board.SCL, board.SDA)
            # Tenta endereço 0x29 (padrão Adafruit) e 0x28
            for addr in [0x29, 0x28]:
                try:
                    self.imu = adafruit_bno055.BNO055_I2C(self.i2c, address=addr)
                    self._bno_addr = addr
                    print(f"[OK] BNO055 Inicializado com sucesso no endereço 0x{addr:02X}!")
                    break
                except Exception:
                    pass
            if not self.imu:
                raise RuntimeError("BNO055 não respondeu nos endereços 0x29 nem 0x28.")
        except Exception as e:
            print(f"Erro ao inicializar sensor BNO055: {e}")
            self.connected = False
            return False

        # Inicia a thread de alta velocidade do GPS
        if self.ser:
            self.configure_gps_high_rate()
            self._thread_running = True
            self._gps_thread = threading.Thread(target=self._gps_reader_loop, daemon=True)
            self._gps_thread.start()

        self.connected = True
        return True

    def get_data(self):
        """Obtem a telemetria consolidada combinando leituras inerciais do BNO055 e dados de navegacao do GPS."""
        if not self.connected:
            return {"connected": False}

        # 1. Recupera as leituras do GPS da thread de background (<0.001 ms)
        with self._data_lock:
            cur_lat = self.last_lat
            cur_lon = self.last_lon
            cur_spd = self.last_speed
            cur_alt = self.last_alt
            cur_trk = self.last_track
            satellites = self.satellites
            gps_qual = self.gps_quality

        # 2. Leitura da IMU BNO055 (Orientação com Calibração Aplicada)
        now = time.time()
        if now - self.last_cal_check > 1.0:
            self.last_cal_check = now
            cal_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sensor_calibration.json")
            if os.path.exists(cal_path):
                try:
                    mt = os.path.getmtime(cal_path)
                    if mt != self.cal_mtime:
                        self.cal_mtime = mt
                        with open(cal_path, "r", encoding="utf-8") as f:
                            self.calibration = json.load(f)
                        print(f"[CALIBRAÇÃO] Atualizada dinamicamente em voo: {self.calibration}")
                except Exception:
                    pass

        pitch = self.last_pitch
        bank = self.last_bank
        heading = self.last_heading if self.last_heading is not None else cur_trk

        if self.imu:
            try:
                euler = self.imu.euler
                if euler and euler[0] is not None:
                    raw_hdg = float(euler[0])
                    raw_roll = float(euler[1]) if euler[1] is not None else 0.0
                    raw_pitch = float(euler[2]) if euler[2] is not None else 0.0

                    true_pitch = raw_roll
                    true_bank  = -raw_pitch
                    self.last_true_pitch = true_pitch
                    self.last_true_bank  = true_bank
                    self.last_raw_heading = raw_hdg

                    b_offset = self.calibration.get("bank_offset", 0.0)
                    p_offset = self.calibration.get("pitch_offset", 0.0)
                    h_offset = self.calibration.get("heading_offset", 0.0)

                    inv_bank = -1.0 if self.calibration.get("invert_bank", False) else 1.0
                    inv_pitch = -1.0 if self.calibration.get("invert_pitch", False) else 1.0
                    hdg_180 = 180.0 if self.calibration.get("invert_heading", False) else 0.0

                    pitch = (true_pitch - p_offset) * inv_pitch
                    bank = (true_bank - b_offset) * inv_bank
                    heading = (raw_hdg + hdg_180 + h_offset + self._aid_offset) % 360.0
                    heading = self._apply_heading_aid(heading, cur_spd, cur_trk, now)

                    self.last_pitch = pitch
                    self.last_bank = bank
                    self.last_heading = heading
                    self._imu_err_count = 0
            except Exception:
                self._imu_err_count = getattr(self, '_imu_err_count', 0) + 1
                if self._imu_err_count > 15:
                    try:
                        for addr in [getattr(self, '_bno_addr', 0x29), 0x28, 0x29]:
                            try:
                                self.imu = adafruit_bno055.BNO055_I2C(self.i2c, address=addr)
                                self._bno_addr = addr
                                self._imu_err_count = 0
                                break
                            except Exception:
                                pass
                    except Exception:
                        pass

        # Estabilização de Rumo no Solo (Regra de Aviônica Real), agora sem salto de fonte:
        # em movimento usa o COG do GPS; devagar/parado converge suavemente para a proa.
        track = self._resolve_track(heading, cur_spd, cur_trk, now)

        true_heading = heading
        true_track = track
        mag_var = 0.0

        return {
            "connected": True,
            "lat": cur_lat,
            "lon": cur_lon,
            "alt": cur_alt,
            "speed": cur_spd,
            "heading": heading,
            "track": track,
            "true_heading": true_heading,
            "true_track": true_track,
            "mag_var": mag_var,
            "pitch": pitch,
            "bank": bank,
            "aoa": 0.0,
            "vsi": 0.0,
            "satellites": satellites,
            "gps_quality": gps_qual,
            "gps_fix": bool(satellites >= 4 and gps_qual > 0),
            "is_real_pi": True
        }

    def disconnect(self):
        """Para a thread de segundo plano do GPS e fecha a comunicacao serial de hardware."""
        self._thread_running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.connected = False


class UDPSource(TelemetrySource):
    """Fonte de dados que recebe pacotes JSON de telemetria via UDP da rede local (porta 5005)."""
    def __init__(self, port=5005):
        """Inicializa o socket UDP nao-bloqueante na porta especificada para recepcao de telemetria."""
        super().__init__()
        self.port = port
        self.sock = None
        self.last_data = {
            "connected": True,
            "lat": -23.6277,
            "lon": -46.6546,
            "alt": 2600.0,
            "speed": 0.0,
            "heading": 0.0,
            "track": 0.0,
            "true_heading": 0.0,
            "true_track": 0.0,
            "mag_var": 0.0,
            "pitch": 0.0,
            "bank": 0.0,
            "aoa": 0.0,
            "vsi": 0.0
        }
        self.last_packet_time = 0

    def connect(self):
        """Cria e associa (bind) o socket UDP a porta de rede local, ativando o modo nao-bloqueante."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(("", self.port))
            self.sock.setblocking(False)
            print(f"[OK] Receptor UDP ativo aguardando telemetria na porta {self.port}...")
            self.connected = True
            return True
        except Exception as e:
            print(f"[ERRO] Falha ao inicializar porta UDP {self.port}: {e}")
            self.connected = False
            return False

    def get_data(self):
        """Esvazia o buffer de rede UDP, deserializa os pacotes JSON mais recentes e aplica a calibracao de atitude."""
        if not self.connected or not self.sock:
            return {"connected": False}
        
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
                if data:
                    payload = json.loads(data.decode("utf-8"))
                    self.last_data.update(payload)
                    self.last_data["connected"] = True
                    self.last_packet_time = time.time()
            except (BlockingIOError, socket.error):
                break
            except Exception:
                break

        if self.last_packet_time > 0 and (time.time() - self.last_packet_time > 3.0):
            self.last_data["connected"] = False

        if not self.last_data.get("connected"):
            return self.last_data

        self.check_calibration_update()

        raw_p = self.last_data.get("_raw_pitch", self.last_data.get("pitch", 0.0))
        raw_b = self.last_data.get("_raw_bank", self.last_data.get("bank", 0.0))
        raw_h = self.last_data.get("_raw_heading", self.last_data.get("heading", 0.0))

        self.last_true_pitch = raw_p
        self.last_true_bank = raw_b
        self.last_raw_heading = raw_h

        cal_p, cal_b, cal_h = self.apply_attitude_calibration(raw_p, raw_b, raw_h)

        res = dict(self.last_data)
        res["pitch"] = cal_p
        res["bank"] = cal_b
        res["heading"] = cal_h
        res["satellites"] = self.last_data.get("satellites", 12)
        res["gps_quality"] = self.last_data.get("gps_quality", 1)
        res["gps_fix"] = True
        res["is_real_pi"] = False
        res["_raw_pitch"] = raw_p
        res["_raw_bank"] = raw_b
        res["_raw_heading"] = raw_h
        return res

    def disconnect(self):
        """Fecha o descritor de socket UDP e desativa a flag de conexao."""
        if self.sock:
            self.sock.close()
        self.connected = False


class MockSource(TelemetrySource):
    """Fonte de dados simulada para desenvolvimento e testes locais."""
    def __init__(self):
        """Inicializa o simulador interno de telemetria com coordenadas e parametros de atitude iniciais."""
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
        """Ativa a geracao continua de dados sinteticos de voo para testes locais."""
        self.connected = True
        self.last_time = time.time()
        print("Mock de telemetria conectado!")
        return True

    def get_data(self):
        """Computa a progressao cinematica da aeronave (latitude, longitude, atitude, IAS e VSI) em funcao do tempo decorrido."""
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
            "bank": self.bank,
            "aoa": 2.5 + random.uniform(-0.1, 0.1),
            "vsi": 0.0,
            "satellites": 12,
            "gps_quality": 1,
            "gps_fix": True,
            "is_real_pi": False
        }

# ============================================================================

# 5. CO-PILOTO NEURAL DE REINTERCEPTAÇÃO COM IA LOCAL QWEN 2.5 (naveg_ai_copilot.py)
# ============================================================================
class NavegAICopilot:
    """
    Co-Piloto Inteligente do NAVEG-AI alimentado por IA local (Qwen 2.5 via Ollama).
    - Opera em Thread assíncrona desacoplada do loop do Pygame (mantém 30 FPS cravados).
    - Incorpora análise dinâmica de taxa de aproximação (Closure Rate) e IAS para evitar overshoot/ziguezague.
    - Aplica Proteção de Envelope Determinística (Guardrails): limite de 20° e afunilamento de 4 NM.
    - Totalmente tolerante a falhas (fallback instantâneo caso a IA demore ou falhe).
    """

    def __init__(self, ollama_url="http://127.0.0.1:11434", model=None, update_interval=1.5):
        """Inicializa o co-piloto neural, detecta o modelo Qwen no servidor Ollama e lanca a thread de inferencia."""
        self.ollama_base = ollama_url.rstrip('/')
        self.ollama_url = f"{self.ollama_base}/api/generate"
        self.update_interval = update_interval
        
        # Auto-detecta o melhor modelo disponível se não especificado
        if model:
            self.model = model
        else:
            self.model = self._detect_model()
            
        print(f"[NAVEG-AI] Co-Piloto Neural ativado utilizando o modelo: {self.model}")
        
        # Estado interno protegido por Lock
        self._lock = threading.Lock()
        self._latest_telemetry = None
        self._latest_guidance = None
        self._is_running = True
        self._last_call_time = 0.0
        
        # Slew-Rate Limiter (Amortecimento gradual grau a grau para o Flight Director)
        self._smooth_target_heading = None
        self._last_smooth_time = None
        self.slew_rate_dps = 3.5  # Graus por segundo (acompanha curva padrão de 3°/s de forma contínua)
        self._latest_qwen_decision = {}
        
        # Histórico para cálculo de taxa de fechamento (Closure Rate)
        self._last_xtk = None
        self._last_xtk_time = None
        self._closure_rate_kt = 0.0  # nós de velocidade lateral em direção ao eixo
        
        # Status de conexão imediata com o Ollama
        self._ai_online = self._check_ollama_available()
        self._consecutive_ai_failures = 0
        
        # Inicia a Thread em segundo plano
        self._worker_thread = threading.Thread(target=self._ai_worker_loop, daemon=True)
        self._worker_thread.start()

    def _check_ollama_available(self):
        """Verifica de forma rápida e não bloqueante se o servidor Ollama está respondendo."""
        try:
            req = urllib.request.Request(f"{self.ollama_base}/api/tags")
            with urllib.request.urlopen(req, timeout=2.5) as resp:
                return resp.status == 200
        except Exception:
            return False

    def is_online(self):
        """Informa se a IA local está conectada e operacional."""
        with self._lock:
            return self._ai_online

    def get_smooth_heading(self):
        """Retorna a proa contínua (float) para movimentação perfeitamente suave da barra do Flight Director."""
        with self._lock:
            return self._smooth_target_heading

    def _detect_model(self):
        """Detecta se qwen2.5:1.5b ou qwen2.5:0.5b está disponível."""
        try:
            req = urllib.request.Request(f"{self.ollama_base}/api/tags")
            with urllib.request.urlopen(req, timeout=2.0) as resp:
                data = json.loads(resp.read().decode('utf-8'))
                models = [m.get("name", "") for m in data.get("models", [])]
                for m in models:
                    if "1.5b" in m:
                        return m
                for m in models:
                    if "0.5b" in m:
                        return m
                if models:
                    return models[0]
        except Exception:
            pass
        return "qwen2.5:1.5b"

    def stop(self):
        """Sinaliza o encerramento seguro da thread de processamento da rede neural."""
        # Sinaliza o encerramento seguro do loop de inferencia neural
        self._is_running = False

    def update_telemetry(self, runway_course, current_heading, xtk_nm, dist_nm, ias_kt, v_phase=0, is_active=True):
        """
        Recebe a telemetria do loop principal a 30 FPS sem travar o Pygame.
        """
        now = time.time()
        
        # Calcula taxa de variação do XTK (closure rate)
        with self._lock:
            if self._last_xtk is not None and self._last_xtk_time is not None:
                dt = now - self._last_xtk_time
                if dt >= 0.5:
                    d_xtk = xtk_nm - self._last_xtk
                    # Se XTK diminui em módulo, estamos nos aproximando do centro
                    # Taxa em nós (NM por hora)
                    rate_nm_hr = (d_xtk / (dt / 3600.0))
                    self._closure_rate_kt = rate_nm_hr
                    self._last_xtk = xtk_nm
                    self._last_xtk_time = now
            else:
                self._last_xtk = xtk_nm
                self._last_xtk_time = now

            self._latest_telemetry = {
                "runway_course": float(runway_course),
                "current_heading": float(current_heading),
                "xtk_nm": float(xtk_nm),
                "dist_nm": float(dist_nm) if dist_nm is not None else 10.0,
                "ias_kt": float(ias_kt) if ias_kt is not None else 100.0,
                "v_phase": int(v_phase),
                "is_active": bool(is_active),
                "closure_rate_kt": self._closure_rate_kt,
                "timestamp": now
            }
            if self._latest_guidance is None and is_active:
                self._latest_guidance = self._apply_envelope_guardrails({}, self._latest_telemetry)

    def get_guidance(self, fallback_heading=None):
        """
        Retorna a decisão atual da IA com proteção de envelope de segurança
        e filtro de transição suave grau a grau (Slew-Rate Limiter).
        Avalia continuamente a 30 FPS a posição em relação ao funil de captura,
        integrando a cognição da Qwen em segundo plano.
        """
        now = time.time()
        with self._lock:
            ai_online = self._ai_online
            telem = self._latest_telemetry
            qwen_dec = self._latest_qwen_decision or {}

            # Avalia continuamente a 30 FPS com a telemetria em tempo real
            if telem and telem.get("is_active"):
                eval_guidance = self._apply_envelope_guardrails(qwen_dec, telem)
                self._latest_guidance = eval_guidance
                raw_target = float(eval_guidance.get("target_heading_float", eval_guidance.get("target_heading", 0.0)))
            elif fallback_heading is not None:
                raw_target = float(fallback_heading)
                eval_guidance = {"target_heading": int(raw_target), "reason": "Modo manual/fallback.", "valid": True}
            else:
                raw_target = 0.0
                eval_guidance = {"target_heading": 0, "reason": "Aguardando telemetria.", "valid": False}

            # Inicialização do filtro suave na primeira chamada
            if self._smooth_target_heading is None:
                self._smooth_target_heading = raw_target
                self._last_smooth_time = now

            dt = max(0.001, min(0.5, now - (self._last_smooth_time or now)))
            self._last_smooth_time = now

            # Menor diferença angular (-180° a +180°)
            diff = (raw_target - self._smooth_target_heading + 180.0) % 360.0 - 180.0

            # Se for transição abrupta de perna/modo (> 30°), transita rápido a 25°/s;
            # Para ajuste de interceptação e rollout suave, transita a 3.5°/s (acompanha perfeitamente curva padrão)
            if abs(diff) > 30.0:
                rate = 25.0
            else:
                rate = self.slew_rate_dps

            max_step = rate * dt
            if abs(diff) <= max_step:
                self._smooth_target_heading = raw_target
            else:
                self._smooth_target_heading = (self._smooth_target_heading + math.copysign(max_step, diff)) % 360.0

            int_target = int(round(self._smooth_target_heading)) % 360

            res_guidance = dict(eval_guidance) if (eval_guidance and eval_guidance.get("valid")) else {
                "target_heading": int_target,
                "ai_active": ai_online,
                "reason": "Calculado via envelope seguro."
            }
            res_guidance["target_heading"] = int_target
            res_guidance["target_heading_float"] = self._smooth_target_heading
            res_guidance["smooth_heading"] = self._smooth_target_heading
            res_guidance["ai_active"] = ai_online

            return int_target, res_guidance

    def _apply_envelope_guardrails(self, raw_decision, telem):
        """
        PROTEÇÃO DE ENVELOPE DETERMINÍSTICA E FUNIL DE CAPTURA ANTECIPADA:
        Garante alinhamento suave com o eixo da pista sem overshoot nem efeito pêndulo.
        - Funil de aproximação reduzido de 4.0 NM para 2.0 NM conforme solicitação do piloto.
        - Zona de Rollout nos últimos ~130 metros (0.07 NM) do eixo.
        - Corte assertivo (mínimo 10° a 18°) para qualquer desvio >= 0.07 NM.
        - Proteção contra overshoot por ângulo de travessia (se cruzando o eixo com erro de proa > 3°,
          não entra em zona morta e comanda corte corretivo para anular a velocidade lateral).
        """
        course = float(telem.get("runway_course", 0.0))
        xtk = float(telem.get("xtk_nm", 0.0))
        dist = float(telem.get("dist_nm", 10.0))
        closure = float(telem.get("closure_rate_kt", 0.0))
        cur_hdg = float(telem.get("current_heading", course))
        
        abs_xtk = abs(xtk)
        hdg_diff_signed = (cur_hdg - course + 180.0) % 360.0 - 180.0
        abs_hdg_err = abs(hdg_diff_signed)

        # 1. ESTABILIZAÇÃO NA FINAL CURTA (< 1.5 NM) OU NO EIXO DA PISTA:
        # Se estiver na final curta (< 1.5 NM) e dentro da faixa de alinhamento (< 0.035 NM ~ 65m) com proa aceitável,
        # ou se estiver dentro do corredor central (< 0.020 NM ~ 37m) em qualquer distância com proa aceitável (< 8°):
        # A aeronave ESTÁ NO EIXO! Mantém asas niveladas no curso sagrado da pista!
        is_short_final_stabilized = (dist <= 1.5 and abs_xtk <= 0.035 and abs_hdg_err <= 10.0)
        is_on_centerline = (abs_xtk <= 0.020 and abs_hdg_err <= 8.0)

        if is_short_final_stabilized or is_on_centerline:
            return {
                "target_heading": int(round(course)) % 360,
                "target_heading_float": course,
                "turn": None,
                "intercept_angle": 0.0,
                "reason": "Final curta: aeronave estabilizada no eixo da pista para pouso." if is_short_final_stabilized else "Aeronave estabilizada e alinhada no centro do eixo.",
                "valid": True,
                "ai_active": True
            }

        # 2. Direção física obrigatória da curva para interceptar
        expected_turn = "L" if xtk > 0 else "R"

        # 3. Limite do funil de segurança conforme a distância da cabeceira
        # Afunilamento de 2.0 NM (solicitação do piloto):
        # Fora de 2 NM: corte livre, firme e assertivo de até 25°.
        # Abaixo de 2 NM: afunila de 25° até 8° próximo da pista.
        if dist > 2.0:
            max_allowed_angle = 25.0
        else:
            ratio = max(0.0, min(1.0, dist / 2.0))
            max_allowed_angle = 8.0 + (25.0 - 8.0) * ratio

        # 4. ZONA DE CAPTURA ANTECIPADA E GANHO ASSERTIVO:
        # O rollout só deve começar nos últimos ~130 metros (0.07 NM) do eixo.
        capture_dist = 0.07  # ~130 metros do centro da pista

        if abs_xtk >= capture_dist:
            # Fora da zona de captura imediata: corte assertivo proporcional ao desvio (70°/NM)
            dynamic_cut = max(10.0, abs_xtk * 70.0)
            ideal_angle = min(max_allowed_angle, dynamic_cut)
            expected_turn = "L" if xtk > 0 else "R"
        else:
            # DENTRO DOS ÚLTIMOS 130 METROS (< 0.07 NM):
            # Afunila suavemente de 10° até zero ao atingir o corredor central (0.020 NM)
            ramp = max(0.0, min(1.0, (abs_xtk - 0.020) / (capture_dist - 0.020)))
            pos_cut = min(max_allowed_angle, 10.0) * ramp

            # Proteção contra travessia do eixo em ângulo extremo (> 12° de erro):
            # Só interfere se a aeronave estiver realmente cortando o eixo feito uma flecha!
            if abs_hdg_err > 12.0:
                drift_cut = min(12.0, abs_hdg_err * 0.5)
                ideal_angle = min(max_allowed_angle, max(pos_cut, drift_cut))
                expected_turn = "L" if hdg_diff_signed > 0 else "R"
            else:
                ideal_angle = pos_cut
                expected_turn = "L" if xtk > 0 else "R"

        # 5. Integração com a decisão cognitiva da IA (Qwen 2.5)
        raw_ia_angle = None
        if raw_decision.get("intercept_angle_deg") is not None:
            try:
                ang_val = float(raw_decision.get("intercept_angle_deg"))
                if ang_val > 0.5:
                    raw_ia_angle = ang_val
            except Exception:
                pass

        if raw_ia_angle is None and raw_decision.get("target_heading") is not None:
            try:
                hdg_f = float(raw_decision.get("target_heading"))
                diff = abs((hdg_f - course + 180.0) % 360.0 - 180.0)
                if diff > 0.5:
                    raw_ia_angle = diff
            except Exception:
                pass

        if raw_ia_angle is not None:
            # IA modula suavemente em torno do cálculo ótimo (±2.5°)
            ia_angle = max(ideal_angle - 2.5, min(ideal_angle + 2.5, raw_ia_angle))
            # Garante que fora da zona de captura o corte NUNCA seja tímido (< 8°)
            if abs_xtk >= capture_dist:
                ia_angle = max(8.0, ia_angle)
        else:
            ia_angle = ideal_angle

        # Garante respeito estrito ao limite do funil de segurança
        ia_angle = max(0.0, min(ia_angle, max_allowed_angle))

        # 6. Calcula a proa ordenada dentro do envelope
        if expected_turn == "L":
            safe_target_heading = (course - ia_angle + 360.0) % 360.0
        else:
            safe_target_heading = (course + ia_angle) % 360.0

        # 7. Mensagem explicativa cognitiva da IA para o piloto
        ia_reason = raw_decision.get("reason")
        if not ia_reason:
            lado_pt = "direita" if expected_turn == "R" else "esquerda"
            if abs_xtk < capture_dist:
                ia_reason = f"Antecipando alinhamento a {abs_xtk:.2f} NM do eixo. Aliviando corte para proa {int(round(safe_target_heading)):03}° para evitar overshoot."
            else:
                ia_reason = f"Interceptando eixo a {abs_xtk:.2f} NM. Mantendo corte para a proa {int(round(safe_target_heading)):03}°."

        return {
            "target_heading": int(round(safe_target_heading)) % 360,
            "target_heading_float": safe_target_heading,
            "turn": expected_turn,
            "intercept_angle": round(ia_angle, 1),
            "reason": ia_reason,
            "valid": True,
            "ai_active": True
        }

    def _ai_worker_loop(self):
        """
        Loop em background: chama a Qwen sem impactar os 30 FPS do Pygame.
        """
        last_health_check = 0.0
        while self._is_running:
            time.sleep(0.1)
            now = time.time()
            
            # Verificação periódica de saúde da IA a cada 4 segundos
            if now - last_health_check >= 4.0:
                last_health_check = now
                ollama_ok = self._check_ollama_available()
                with self._lock:
                    if ollama_ok:
                        if not self._ai_online:
                            self._ai_online = True
                            self._consecutive_ai_failures = 0
                        if not self.model or self.model == "qwen2.5:0.5b":
                            detected = self._detect_model()
                            if detected:
                                self.model = detected
                    else:
                        if self._consecutive_ai_failures >= 3:
                            self._ai_online = False

            with self._lock:
                telem = self._latest_telemetry
                
            if not telem or not telem.get("is_active"):
                continue

            # Só reavalia a cada update_interval
            if now - self._last_call_time < self.update_interval:
                continue

            # Só aciona na interceptação, final ou desvio significativo
            v_phase = telem.get("v_phase", 0)
            xtk = telem.get("xtk_nm", 0.0)
            
            # Se não estiver em modo de interceptação/final e o XTK for irrelevante, poupa CPU
            if abs(xtk) < 0.015 and v_phase not in (6, 7):
                continue

            self._last_call_time = now
            self._query_qwen(telem)

    def _query_qwen(self, telem):
        """
        Monta prompt otimizado e envia para a Qwen via Ollama.
        """
        course = telem["runway_course"]
        xtk = telem["xtk_nm"]
        dist = telem["dist_nm"]
        ias = telem["ias_kt"]
        closure = telem["closure_rate_kt"]
        cur_hdg = telem.get("current_heading", course)
        
        lado_str = "DIREITA" if xtk > 0 else "ESQUERDA"
        tendencia = "aproximando rápido do centro" if (xtk > 0 and closure < -0.5) or (xtk < 0 and closure > 0.5) else "estável ou afastando"
        
        prompt = (
            f"Você é o Co-Piloto Neural do NAVEG-AI especializado em aproximação final e interceptação suave de pista.\n"
            f"Telemetria de voo atual:\n"
            f"- Eixo desejado da pista: {course:.0f}°\n"
            f"- Desvio lateral (XTK): {abs(xtk):.3f} NM à {lado_str} do eixo\n"
            f"- Proa atual da aeronave: {cur_hdg:.0f}°\n"
            f"- Tendência de trajetória: aeronave está {tendencia} ({abs(closure):.1f} kt de taxa lateral)\n"
            f"- Distância da cabeceira: {dist:.1f} NM | IAS: {ias:.0f} kt\n"
            f"Critérios de Pilotagem e Rollout Suave:\n"
            f"1. Se desvio >= 0.07 NM: comande corte ASSERTIVO e firme (10° a 18°) para fechar o desvio rápido.\n"
            f"2. Se desvio < 0.07 NM: antecipe o alinhamento com a pista, reduzindo gradualmente o ângulo de corte (para 4° a 8°) para rollout suave sem overshoot.\n"
            f"3. Se já no eixo (desvio < 0.01 NM): corte de 0° mantendo o curso da pista.\n"
            f"Responda APENAS JSON: {{\"intercept_angle_deg\": float, \"reason\": \"breve explicação ao piloto em português\"}}"
        )

        payload = {
            "model": self.model,
            "prompt": prompt,
            "stream": False,
            "format": "json",
            "keep_alive": "60m",
            "options": {
                "temperature": 0.0,
                "num_predict": 80
            }
        }

        try:
            req_data = json.dumps(payload).encode('utf-8')
            req = urllib.request.Request(self.ollama_url, data=req_data, headers={'Content-Type': 'application/json'})
            with urllib.request.urlopen(req, timeout=5.0) as resp:
                res_json = json.loads(resp.read().decode('utf-8'))
                raw_text = res_json.get("response", "{}")
                parsed = {}
                try:
                    parsed = json.loads(raw_text)
                except Exception:
                    # Fallback com regex caso o JSON venha truncado ou imperfeito
                    import re
                    m = re.search(r'"intercept_angle_deg"\s*:\s*([0-9.]+)', raw_text)
                    if m:
                        parsed = {"intercept_angle_deg": float(m.group(1))}
                    else:
                        parsed = {}
                
                safe_guidance = self._apply_envelope_guardrails(parsed, telem)
                with self._lock:
                    self._latest_qwen_decision = parsed
                    self._latest_guidance = safe_guidance
                    self._consecutive_ai_failures = 0
                    self._ai_online = True
        except Exception:
            # Em caso de atraso transitório, calcula guardrails matemáticos e mantém IA online (tolerância a picos do MSFS)
            safe_guidance = self._apply_envelope_guardrails(self._latest_qwen_decision or {}, telem)
            with self._lock:
                self._latest_guidance = safe_guidance
                self._consecutive_ai_failures += 1
                if self._consecutive_ai_failures >= 5:
                    self._ai_online = False

# ============================================================================
# 6. INTERFACE GRÁFICA PFD E FLIGHT DIRECTOR (pfd_gui.py)

# ============================================================================
# Cores do Sistema
WHITE = (240, 240, 240)
BLACK = (10, 10, 10)
GRAY = (60, 60, 65)
LIGHT_GRAY = (150, 150, 150)
MAGENTA = (255, 0, 255)
SKY = (65, 135, 210)
GROUND = (130, 80, 40)
GREEN = (40, 210, 90)
BTN_ON_GREEN = (25, 140, 65)  # Verde mais escuro e sóbrio para botões em estado ON
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


class TouchKeypad:
    """Teclado numérico touch compacto integrado para cockpit / Raspberry Pi."""
    def __init__(self, gui):
        """Inicializa o teclado virtual touch, configurando fontes, dimensoes e cache de superficies graficas pre-renderizadas."""
        self.gui = gui
        self.active = False
        self.field_name = ""
        self.title = ""
        self.value_str = ""
        self.quick_type = "altitude"  # "altitude", "miles", "course"
        self.rect = pygame.Rect(700, 246, 360, 440)
        self.buttons = {}
        self.visor_rect = pygame.Rect(715, 290, 330, 44)
        self.dragging = False
        self.drag_offset = (0, 0)
        self.user_dragged = False
        self.open_time = 0.0
        
        # Cache de superfícies estáticas pré-renderizadas (Altíssima responsividade)
        self._key_surfs = {}
        for k in ["0","1","2","3","4","5","6","7","8","9","."]:
            self._key_surfs[k] = self.gui.f_btn.render(k, True, (240, 240, 240))
        self._key_surfs["backspace"] = self.gui.f_btn.render("<-", True, (240, 240, 240))
        self._x_surf = self.gui.f_label.render("X", True, (255, 255, 255))
        self._canc_surf = self.gui.f_field_label.render("CANCELAR", True, (240, 240, 240))
        self._com_surf = self.gui.f_field_label.render("ATIVAR", True, (255, 255, 255))
        self._shadow_surf = pygame.Surface((360, 440), pygame.SRCALPHA)
        self._shadow_surf.fill((0, 0, 0, 150))
        self._title_surf = None
        self._quick_surfs = []

    def open(self, field_name, title, initial_value, quick_type="altitude", pos=(700, 246)):
        """Abre o teclado na tela com o valor inicial e o tipo de campo numerico selecionado."""
        self.active = True
        self.dragging = False
        self.field_name = field_name
        self.title = title
        self.value_str = str(initial_value) if initial_value else ""
        self.quick_type = quick_type
        self.open_time = time.time()
        # Se o piloto já posicionou o teclado em outro local da tela, preserva a posição!
        if not self.user_dragged:
            x = max(10, min(pos[0], 1800 - 370))
            y = max(10, min(pos[1], 900 - 450))
            self.rect = pygame.Rect(x, y, 360, 440)
        else:
            self.rect.x = max(10, min(self.rect.x, 1800 - 370))
            self.rect.y = max(10, min(self.rect.y, 900 - 450))
        self._build_buttons()
        # Pré-renderiza elementos do diálogo
        self._title_surf = self.gui.f_field_label.render(self.title, True, (255, 255, 255))
        quick_info = self.get_quick_labels_and_values()
        self._quick_surfs = [self.gui.f_label.render(qi[0], True, (220, 230, 245)) for qi in quick_info]

    def close(self):
        """Fecha e desativa o teclado virtual na tela."""
        self.active = False
        self.dragging = False
        self.field_name = ""

    def handle_motion(self, mouse_pos):
        """Move o teclado na tela ao arrastar pela barra de título."""
        if not self.active or not self.dragging:
            return False
        new_x = mouse_pos[0] - self.drag_offset[0]
        new_y = mouse_pos[1] - self.drag_offset[1]
        self.rect.x = max(10, min(new_x, 1800 - self.rect.w - 10))
        self.rect.y = max(10, min(new_y, 900 - self.rect.h - 10))
        self._build_buttons()
        return True

    def handle_mouse_up(self):
        """Solta o teclado ao soltar o mouse/toque."""
        if self.dragging:
            self.dragging = False
            return True
        return False

    def _build_buttons(self):
        """Calcula o posicionamento e os retangulos de colisao (hitboxes) de todas as teclas numericas e botoes de controle."""
        self.buttons = {}
        rx, ry = self.rect.x, self.rect.y
        self.buttons["close"] = pygame.Rect(rx + self.rect.w - 42, ry + 8, 34, 30)
        self.visor_rect = pygame.Rect(rx + 15, ry + 44, self.rect.w - 30, 44)
        
        qw = 76
        qh = 36
        qy = ry + 96
        for i, q_name in enumerate(["quick_1", "quick_2", "quick_3", "quick_4"]):
            self.buttons[q_name] = pygame.Rect(rx + 15 + i * (qw + 8), qy, qw, qh)
            
        nw = 98
        nh = 46
        ny_start = ry + 140
        grid = [
            ["7", "8", "9"],
            ["4", "5", "6"],
            ["1", "2", "3"],
            [".", "0", "backspace"]
        ]
        for row_idx, row in enumerate(grid):
            for col_idx, key in enumerate(row):
                self.buttons[f"key_{key}"] = pygame.Rect(
                    rx + 15 + col_idx * (nw + 13),
                    ny_start + row_idx * (nh + 8),
                    nw, nh
                )
                
        by = ry + 368
        self.buttons["cancel"] = pygame.Rect(rx + 15, by, 140, 50)
        self.buttons["commit"] = pygame.Rect(rx + 170, by, 175, 50)

    def get_quick_labels_and_values(self):
        """Retorna a lista de valores dos botoes de atalho rapido de acordo com o modo ativo (altitude, milhas ou curso)."""
        if self.quick_type == "altitude":
            # Botão de altitude com +/- 25 FT a pedido do Comandante!
            return [("-100", -100), ("-25", -25), ("+25", +25), ("+100", +100)]
        elif self.quick_type == "miles":
            return [("-0.5", -0.5), ("-0.1", -0.1), ("+0.1", +0.1), ("+0.5", +0.5)]
        elif self.quick_type == "course":
            return [("-10", -10), ("-1", -1), ("+1", +1), ("+10", +10)]
        return [("-10", -10), ("-1", -1), ("+1", +1), ("+10", +10)]

    def handle_click(self, mouse_pos):
        """Processa o toque ou clique do mouse nas teclas virtuais, atualizando o valor no visor digital."""
        if not self.active:
            return {"action": "none"}
            
        if not self.rect.collidepoint(mouse_pos):
            # Debounce contra duplo-toque acidental com dedo:
            # Se acabou de abrir a menos de 500ms, ignora o segundo toque fora do teclado!
            if time.time() - self.open_time < 0.50:
                return {"action": "none"}
            self.close()
            return {"action": "outside"}
            
        # Botão Fechar [X]
        if self.buttons.get("close", pygame.Rect(0,0,0,0)).collidepoint(mouse_pos):
            self.close()
            return {"action": "close"}

        # Barra de Título Arrastável (topo até o visor, exceto botão [X])
        title_bar_rect = pygame.Rect(self.rect.x, self.rect.y, self.rect.w - 46, 44)
        if title_bar_rect.collidepoint(mouse_pos):
            self.dragging = True
            self.user_dragged = True
            self.drag_offset = (mouse_pos[0] - self.rect.x, mouse_pos[1] - self.rect.y)
            return {"action": "drag_start"}
            
        # Botão Cancelar
        if self.buttons.get("cancel", pygame.Rect(0,0,0,0)).collidepoint(mouse_pos):
            self.close()
            return {"action": "close"}
            
        # Botão Ativar [COMMIT]
        if self.buttons.get("commit", pygame.Rect(0,0,0,0)).collidepoint(mouse_pos):
            val = self.value_str
            field = self.field_name
            self.close()
            return {"action": "commit", "field": field, "value": val}
            
        # Quick adjust buttons (+25, -25, +0.1, -0.1, etc.)
        quick_info = self.get_quick_labels_and_values()
        for i, q_name in enumerate(["quick_1", "quick_2", "quick_3", "quick_4"]):
            if self.buttons.get(q_name, pygame.Rect(0,0,0,0)).collidepoint(mouse_pos):
                delta = quick_info[i][1]
                if self.quick_type == "altitude":
                    try:
                        cur = float(self.value_str) if self.value_str else 0.0
                    except ValueError:
                        cur = 0.0
                    new_val = max(0.0, cur + delta)
                    self.value_str = str(int(round(new_val)))
                elif self.quick_type == "miles":
                    try:
                        cur = float(self.value_str) if self.value_str else 0.3
                    except ValueError:
                        cur = 0.3
                    new_val = max(0.05, round(cur + delta, 2))
                    self.value_str = f"{new_val:.2f}".rstrip('0').rstrip('.')
                elif self.quick_type == "course":
                    try:
                        cur = int(self.value_str) if self.value_str.isdigit() else 0
                    except ValueError:
                        cur = 0
                    new_val = (cur + delta) % 360
                    self.value_str = f"{int(new_val):03}"
                return {"action": "update", "value": self.value_str}
                
        # Numpad keys (0-9)
        for digit in ["0","1","2","3","4","5","6","7","8","9"]:
            if self.buttons.get(f"key_{digit}", pygame.Rect(0,0,0,0)).collidepoint(mouse_pos):
                if self.quick_type == "course" and len(self.value_str) >= 3:
                    pass
                elif self.quick_type == "miles" and len(self.value_str) >= 5:
                    pass
                elif self.quick_type == "altitude" and len(self.value_str) >= 5:
                    pass
                else:
                    self.value_str += digit
                return {"action": "update", "value": self.value_str}
                
        if self.buttons.get("key_.", pygame.Rect(0,0,0,0)).collidepoint(mouse_pos):
            if "." not in self.value_str and len(self.value_str) < 5:
                self.value_str += "." if self.value_str else "0."
            return {"action": "update", "value": self.value_str}
            
        if self.buttons.get("key_backspace", pygame.Rect(0,0,0,0)).collidepoint(mouse_pos):
            self.value_str = self.value_str[:-1]
            return {"action": "update", "value": self.value_str}
            
        # Qualquer outro ponto dentro do teclado consome o clique
        return {"action": "consumed"}

    def draw(self, surface, mouse_pos):
        """Renderiza o teclado virtual na tela, incluindo painel de fundo, visor digital e teclas com feedback visual."""
        if not self.active:
            return
            
        # Sombra suave pré-alocada (Zero custo de alocação por frame)
        surface.blit(self._shadow_surf, (self.rect.x + 6, self.rect.y + 6))
        
        # Fundo do Teclado
        pygame.draw.rect(surface, (22, 24, 30), self.rect, border_radius=12)
        border_col = (0, 210, 255) if self.dragging else (50, 130, 220)
        pygame.draw.rect(surface, border_col, self.rect, 2, border_radius=12)
        
        # Barra de Titulo (Área Arrastável com Vermelho de Alto Contraste)
        title_bar_rect = pygame.Rect(self.rect.x, self.rect.y, self.rect.w, 42)
        if self.dragging:
            pygame.draw.rect(surface, (230, 40, 40), title_bar_rect, border_top_left_radius=12, border_top_right_radius=12)
        else:
            pygame.draw.rect(surface, (185, 25, 25), title_bar_rect, border_top_left_radius=12, border_top_right_radius=12)

        # Linha inferior divisória do cabeçalho
        pygame.draw.line(surface, (255, 120, 120), (self.rect.x, self.rect.y + 42), (self.rect.x + self.rect.w, self.rect.y + 42), 2)

        if self._title_surf:
            surface.blit(self._title_surf, (self.rect.x + 14, self.rect.y + 11))
        
        # Indicador visual de arrasto (pontos grip antes do botão X)
        grip_x = self.rect.x + self.rect.w - 82
        for gx in (grip_x, grip_x + 8, grip_x + 16):
            for gy in (self.rect.y + 13, self.rect.y + 20, self.rect.y + 27):
                pygame.draw.circle(surface, (255, 255, 255) if not self.dragging else (255, 240, 180), (gx, gy), 2)
        
        # Botao [X] Fechar (com alto contraste sobre a barra vermelha)
        c_rect = self.buttons["close"]
        c_hover = c_rect.collidepoint(mouse_pos)
        pygame.draw.rect(surface, (255, 70, 70) if c_hover else (35, 12, 12), c_rect, border_radius=6)
        pygame.draw.rect(surface, (255, 200, 200), c_rect, 1, border_radius=6)
        if self._x_surf:
            surface.blit(self._x_surf, (c_rect.centerx - self._x_surf.get_width()//2, c_rect.centery - self._x_surf.get_height()//2))
        
        # Visor de Valor
        pygame.draw.rect(surface, (10, 12, 16), self.visor_rect, border_radius=8)
        pygame.draw.rect(surface, (60, 140, 240), self.visor_rect, 2, border_radius=8)
        
        # Texto com cursor piscante
        cursor = "|" if (pygame.time.get_ticks() // 400) % 2 == 0 else ""
        disp_txt = self.value_str + cursor
        v_surf = self.gui.f_data_compact.render(disp_txt, True, (50, 220, 240))
        surface.blit(v_surf, (self.visor_rect.x + 12, self.visor_rect.centery - v_surf.get_height()//2))
        
        # Botoes de Ajuste Rapido (+/- 25 FT ou +/- 0.1 NM)
        for i, q_name in enumerate(["quick_1", "quick_2", "quick_3", "quick_4"]):
            q_rect = self.buttons[q_name]
            q_hover = q_rect.collidepoint(mouse_pos)
            bg_col = (50, 60, 75) if q_hover else (35, 40, 52)
            pygame.draw.rect(surface, bg_col, q_rect, border_radius=6)
            pygame.draw.rect(surface, (80, 100, 130), q_rect, 1, border_radius=6)
            if i < len(self._quick_surfs):
                surface.blit(self._quick_surfs[i], (q_rect.centerx - self._quick_surfs[i].get_width()//2, q_rect.centery - self._quick_surfs[i].get_height()//2))
            
        # Teclado Numerico (0-9, ., <-)
        grid = [
            ["7", "8", "9"],
            ["4", "5", "6"],
            ["1", "2", "3"],
            [".", "0", "backspace"]
        ]
        for row in grid:
            for key in row:
                k_rect = self.buttons[f"key_{key}"]
                k_hover = k_rect.collidepoint(mouse_pos)
                if key == "backspace":
                    bg = (80, 40, 45) if k_hover else (55, 30, 35)
                    border = (180, 60, 70)
                else:
                    bg = (48, 52, 65) if k_hover else (34, 38, 48)
                    border = (65, 72, 88)
                pygame.draw.rect(surface, bg, k_rect, border_radius=8)
                pygame.draw.rect(surface, border, k_rect, 1, border_radius=8)
                k_surf = self._key_surfs.get(key)
                if k_surf:
                    surface.blit(k_surf, (k_rect.centerx - k_surf.get_width()//2, k_rect.centery - k_surf.get_height()//2))
                
        # Botoes de Acao (CANCELAR e ATIVAR)
        btn_cancel = self.buttons["cancel"]
        c_hover = btn_cancel.collidepoint(mouse_pos)
        pygame.draw.rect(surface, (120, 40, 45) if c_hover else (75, 28, 32), btn_cancel, border_radius=8)
        pygame.draw.rect(surface, (180, 60, 65), btn_cancel, 1, border_radius=8)
        if self._canc_surf:
            surface.blit(self._canc_surf, (btn_cancel.centerx - self._canc_surf.get_width()//2, btn_cancel.centery - self._canc_surf.get_height()//2))
        
        btn_commit = self.buttons["commit"]
        com_hover = btn_commit.collidepoint(mouse_pos)
        pygame.draw.rect(surface, (35, 175, 80) if com_hover else (25, 140, 65), btn_commit, border_radius=8)
        pygame.draw.rect(surface, (50, 220, 100), btn_commit, 2, border_radius=8)
        if self._com_surf:
            surface.blit(self._com_surf, (btn_commit.centerx - self._com_surf.get_width()//2, btn_commit.centery - self._com_surf.get_height()//2))


class AttitudePanel:
    """Painel de nivelamento do Horizonte Artificial (Zerar Pitch/Bank de repouso na perna)."""
    def __init__(self, gui):
        """Inicializa o painel touch de nivelamento de atitude e calibração de horizonte."""
        self.gui = gui
        self.active = False
        self.dragging = False
        self.drag_offset = (0, 0)
        self.user_dragged = False
        self.rect = pygame.Rect(30, 200, 540, 365)
        self.buttons = {}
        self.feedback_msg = ""
        self.feedback_time = 0
        
        # Cache de superfícies estáticas pré-renderizadas (Altíssima fluidez e responsividade)
        self._shadow_surf = pygame.Surface((540, 365), pygame.SRCALPHA)
        self._shadow_surf.fill((0, 0, 0, 150))
        self._x_surf = self.gui.f_label.render("X", True, (255, 255, 255))
        self._title_surf = self.gui.f_field_label.render("NIVELAR HORIZONTE / REPOUSO", True, (255, 255, 255))
        self._zero_surf = self.gui.f_field_label.render("ZERAR PITCH & BANK (NIVELAR 0°)", True, (255, 255, 255))
        self._done_surf = self.gui.f_btn.render("CONCLUIR", True, (255, 255, 255))
        
        # Pré-renderização dos botões de alternância (Zero impacto na CPU durante o arrasto)
        self._ip_sim_surf = self.gui.f_field_label.render("INVERTER PITCH: SIM", True, (255, 255, 255))
        self._ip_nao_surf = self.gui.f_field_label.render("INVERTER PITCH: NÃO", True, (255, 255, 255))
        self._ib_sim_surf = self.gui.f_field_label.render("INVERTER BANK: SIM", True, (255, 255, 255))
        self._ib_nao_surf = self.gui.f_field_label.render("INVERTER BANK: NÃO", True, (255, 255, 255))
        self._ih_sim_surf = self.gui.f_field_label.render("INVERTER PROA (180°): SIM", True, (255, 255, 255))
        self._ih_nao_surf = self.gui.f_field_label.render("INVERTER PROA (180°): NÃO", True, (255, 255, 255))

        # Cache dinâmico da leitura de Pitch e Bank
        self._cached_p = None
        self._cached_b = None
        self._cached_read_surf = None

    def open(self):
        """Abre o painel de calibracao de atitude na tela do PFD."""
        self.active = True
        self.dragging = False
        self.feedback_msg = ""
        # Preserva a posição na tela caso o piloto já tenha arrastado antes
        if not self.user_dragged:
            self.rect = pygame.Rect(30, 200, 540, 365)
        else:
            self.rect.x = max(10, min(self.rect.x, 1800 - self.rect.w - 10))
            self.rect.y = max(10, min(self.rect.y, 900 - self.rect.h - 10))
        self._build_buttons()

    def close(self):
        """Fecha o painel de nivelamento de atitude."""
        self.active = False
        self.dragging = False
        self.feedback_msg = ""

    def handle_motion(self, mouse_pos):
        """Move o painel na tela instantaneamente ao arrastar."""
        if not self.active or not self.dragging:
            return False
        new_x = mouse_pos[0] - self.drag_offset[0]
        new_y = mouse_pos[1] - self.drag_offset[1]
        self.rect.x = max(10, min(new_x, 1800 - self.rect.w - 10))
        self.rect.y = max(10, min(new_y, 900 - self.rect.h - 10))
        self._build_buttons()
        return True

    def handle_mouse_up(self):
        """Solta o painel ao soltar o mouse/toque."""
        if self.dragging:
            self.dragging = False
            return True
        return False

    def _build_buttons(self):
        """Gera os retangulos dos botoes de ajuste rapido de pitch e bank."""
        rx, ry = self.rect.x, self.rect.y
        self.buttons["close"] = pygame.Rect(rx + self.rect.w - 44, ry + 7, 36, 32)
        self.buttons["zero_tare"] = pygame.Rect(rx + 20, ry + 94, self.rect.w - 40, 50)
        bw = (self.rect.w - 50) // 2
        self.buttons["toggle_pitch"] = pygame.Rect(rx + 20, ry + 152, bw, 46)
        self.buttons["toggle_bank"] = pygame.Rect(rx + 20 + bw + 10, ry + 152, bw, 46)
        self.buttons["toggle_heading"] = pygame.Rect(rx + 20, ry + 206, self.rect.w - 40, 46)
        self.buttons["done"] = pygame.Rect(rx + 20, ry + 260, self.rect.w - 40, 48)

    def handle_click(self, mouse_pos):
        """Processa os comandos de ajuste (zerar horizonte, incrementar/decrementar offsets de pitch e bank)."""
        if not self.active:
            return {"action": "none"}
        if not self.rect.collidepoint(mouse_pos):
            self.close()
            return {"action": "close"}
        if self.buttons.get("close", pygame.Rect(0,0,0,0)).collidepoint(mouse_pos):
            self.close()
            return {"action": "close"}
        if self.buttons.get("done", pygame.Rect(0,0,0,0)).collidepoint(mouse_pos):
            self.close()
            return {"action": "close"}
        
        # Barra de Cabeçalho Ampla Arrastável (faixa superior inteira até o botão ZERAR, exceto botão [X])
        title_bar_rect = pygame.Rect(self.rect.x, self.rect.y, self.rect.w - 50, 90)
        if title_bar_rect.collidepoint(mouse_pos):
            self.dragging = True
            self.user_dragged = True
            self.drag_offset = (mouse_pos[0] - self.rect.x, mouse_pos[1] - self.rect.y)
            return {"action": "drag_start"}

        if self.buttons.get("zero_tare", pygame.Rect(0,0,0,0)).collidepoint(mouse_pos):
            return {"action": "zero_tare"}
        if self.buttons.get("toggle_pitch", pygame.Rect(0,0,0,0)).collidepoint(mouse_pos):
            return {"action": "toggle_pitch"}
        if self.buttons.get("toggle_bank", pygame.Rect(0,0,0,0)).collidepoint(mouse_pos):
            return {"action": "toggle_bank"}
        if self.buttons.get("toggle_heading", pygame.Rect(0,0,0,0)).collidepoint(mouse_pos):
            return {"action": "toggle_heading"}
        return {"action": "consumed"}

    def draw(self, surface, mouse_pos, pitch, bank, calib):
        """Desenha a janela flutuante do painel de nivelamento com os valores de offset atuais."""
        if not self.active:
            return

        # Sombra suave só quando parado (pula durante o arrasto para fluidez máxima de 60 FPS)
        if not self.dragging:
            surface.blit(self._shadow_surf, (self.rect.x + 6, self.rect.y + 6))

        pygame.draw.rect(surface, (20, 22, 28), self.rect, border_radius=12)
        border_col = (0, 240, 255) if self.dragging else (0, 210, 255)
        pygame.draw.rect(surface, border_col, self.rect, 2, border_radius=12)

        header_rect = pygame.Rect(self.rect.x, self.rect.y, self.rect.w, 46)
        h_color = (35, 160, 85) if self.dragging else (25, 125, 65)
        pygame.draw.rect(surface, h_color, header_rect, border_top_left_radius=12, border_top_right_radius=12)
        pygame.draw.line(surface, (70, 230, 130), (self.rect.x, self.rect.y + 46), (self.rect.x + self.rect.w, self.rect.y + 46), 2)
        
        surface.blit(self._title_surf, (self.rect.x + 16, self.rect.y + 13))

        # Indicador visual de arrasto (grip dots) na barra de cabeçalho
        for gx in range(self.rect.x + self.rect.w - 95, self.rect.x + self.rect.w - 55, 8):
            for gy in (self.rect.y + 18, self.rect.y + 26):
                pygame.draw.circle(surface, (200, 255, 220) if self.dragging else (110, 190, 140), (gx, gy), 2)

        c_rect = self.buttons["close"]
        c_hover = c_rect.collidepoint(mouse_pos)
        pygame.draw.rect(surface, (200, 40, 40) if c_hover else (35, 12, 12), c_rect, border_radius=6)
        surface.blit(self._x_surf, (c_rect.centerx - self._x_surf.get_width()//2, c_rect.centery - self._x_surf.get_height()//2))

        # Leitura de atitude com cache ultra-rápido (evita chamadas redundantes a font.render)
        p_round = round(pitch, 1)
        b_round = round(bank, 1)
        if (p_round != self._cached_p) or (b_round != self._cached_b) or (self._cached_read_surf is None):
            self._cached_p = p_round
            self._cached_b = b_round
            now_str = f"Leitura Atual: Pitch {p_round:+4.1f}°  |  Bank {b_round:+4.1f}°"
            self._cached_read_surf = self.gui.f_label.render(now_str, True, (200, 220, 240))
        surface.blit(self._cached_read_surf, (self.rect.x + 20, self.rect.y + 56))

        if self.feedback_msg and (time.time() - self.feedback_time < 3.0):
            fb_surf = self.gui.f_label.render(self.feedback_msg, True, (50, 255, 120))
            surface.blit(fb_surf, (self.rect.x + 20, self.rect.y + 76))

        # Botão Zerar Pitch & Bank
        btn_z = self.buttons["zero_tare"]
        z_col = (35, 160, 80) if btn_z.collidepoint(mouse_pos) else (25, 130, 65)
        pygame.draw.rect(surface, z_col, btn_z, border_radius=8)
        pygame.draw.rect(surface, (80, 240, 140), btn_z, 2, border_radius=8)
        surface.blit(self._zero_surf, (btn_z.centerx - self._zero_surf.get_width()//2, btn_z.centery - self._zero_surf.get_height()//2))

        # Botão Inverter Pitch
        btn_ip = self.buttons["toggle_pitch"]
        inv_p = calib.get("invert_pitch", False)
        ip_col = (180, 90, 20) if inv_p else (40, 45, 58)
        pygame.draw.rect(surface, ip_col, btn_ip, border_radius=8)
        pygame.draw.rect(surface, (220, 140, 50) if inv_p else (80, 90, 110), btn_ip, 1, border_radius=8)
        ip_surf = self._ip_sim_surf if inv_p else self._ip_nao_surf
        surface.blit(ip_surf, (btn_ip.centerx - ip_surf.get_width()//2, btn_ip.centery - ip_surf.get_height()//2))

        # Botão Inverter Bank
        btn_ib = self.buttons["toggle_bank"]
        inv_b = calib.get("invert_bank", False)
        ib_col = (180, 90, 20) if inv_b else (40, 45, 58)
        pygame.draw.rect(surface, ib_col, btn_ib, border_radius=8)
        pygame.draw.rect(surface, (220, 140, 50) if inv_b else (80, 90, 110), btn_ib, 1, border_radius=8)
        ib_surf = self._ib_sim_surf if inv_b else self._ib_nao_surf
        surface.blit(ib_surf, (btn_ib.centerx - ib_surf.get_width()//2, btn_ib.centery - ib_surf.get_height()//2))

        # Botão Inverter Proa (180°)
        btn_ih = self.buttons["toggle_heading"]
        inv_h = calib.get("invert_heading", False)
        ih_col = (180, 90, 20) if inv_h else (40, 45, 58)
        pygame.draw.rect(surface, ih_col, btn_ih, border_radius=8)
        pygame.draw.rect(surface, (220, 140, 50) if inv_h else (80, 90, 110), btn_ih, 1, border_radius=8)
        ih_surf = self._ih_sim_surf if inv_h else self._ih_nao_surf
        surface.blit(ih_surf, (btn_ih.centerx - ih_surf.get_width()//2, btn_ih.centery - ih_surf.get_height()//2))

        # Botão Concluir
        btn_d = self.buttons["done"]
        d_col = (60, 70, 85) if btn_d.collidepoint(mouse_pos) else (45, 52, 65)
        pygame.draw.rect(surface, d_col, btn_d, border_radius=8)
        pygame.draw.rect(surface, (120, 140, 170), btn_d, 1, border_radius=8)
        surface.blit(self._done_surf, (btn_d.centerx - self._done_surf.get_width()//2, btn_d.centery - self._done_surf.get_height()//2))


class PFDGui:
    """
    Mecanismo principal de renderizacao grafica do Primary Flight Display (PFD) e Head-Up Display (HUD) em Pygame.
    Desenha em tempo real o horizonte artificial, escala de arfagem (pitch ladder), indicador de rolamento, tapes de velocidade e altitude,
    rosa de bussola / HSI, Cross-Track Error (CDI), rampa de Glide Slope sintetica de 50 FT e barras de comando do Flight Director.
    """
    def __init__(self, base_size=(1800, 900)):
        """Inicializa o mecanismo grafico do PFD, carregando fontes tipograficas e definindo todos os retangulos e botoes da interface."""
        self.base_size = base_size
        
        # Inicializa as fontes do cockpit
        self.f_title = pygame.font.SysFont("Segoe UI", 28, bold=True)
        self.f_label = pygame.font.SysFont("Segoe UI", 16, bold=True)
        self.f_data = pygame.font.SysFont("Consolas", 42, bold=True)
        self.f_data_compact = pygame.font.SysFont("Consolas", 28, bold=True)  # Para valores longos (5+ dígitos)
        self.f_msg = pygame.font.SysFont("Segoe UI", 22, italic=True)
        self.f_btn = pygame.font.SysFont("Segoe UI", 22, bold=True)
        self.f_btn_sub = pygame.font.SysFont("Segoe UI", 17, bold=True)
        self.f_btn_big = pygame.font.SysFont("Segoe UI", 24, bold=True)
        self.f_status = pygame.font.SysFont("Segoe UI", 19, bold=True)
        self.f_small = pygame.font.SysFont("Consolas", 14, bold=True)
        self.f_field_label = pygame.font.SysFont("Segoe UI", 18, bold=True)
        self.f_logo = pygame.font.SysFont("Segoe UI", 42, bold=True)
        
        # Teclado Touchscreen Compacto Integrado
        self.touch_keypad = TouchKeypad(self)
        
        # Painel de Nivelamento de Atitude (Zerar Pitch/Bank na Perna)
        self.attitude_panel = AttitudePanel(self)
        
        # Linha 1 - Botão INICIAR/PARAR monitor
        self.btn_monitor = pygame.Rect(600, 100, 350, 54)

        # Linha 1 - Botão LOG (ao lado do INICIAR/PARAR monitor)
        self.btn_log = pygame.Rect(965, 100, 80, 54)
        
        # Linha 1 - Botão LIMPAR RAMPA (ao lado do LOG, visível se houver rampa salva)
        self.btn_clear_gs = pygame.Rect(1055, 100, 115, 54)

        # Linha 2 - Lado Esquerdo: Entrada do Curso Manual (redimensionado para caber GS)
        self.course_input_rect = pygame.Rect(600, 190, 130, 46)
        self.btn_set_course = pygame.Rect(740, 190, 75, 46)

        # Linha 2 - Centro-Esquerdo: Tolerância XTK customizada (redimensionado)
        self.tolerance_input_rect = pygame.Rect(825, 190, 110, 46)
        self.btn_set_tolerance = pygame.Rect(945, 190, 65, 46)

        # Campo para digitar a altitude da pista (elevação do aeródromo)
        self.gs_rwy_alt_input_rect = pygame.Rect(1020, 190, 148, 46)
        
        # Botões do GS Sintético
        self.btn_save_gs = pygame.Rect(1180, 100, 115, 54) # Ao lado do LIMPAR RAMPA, acima do 50FT
        self.btn_start_gs = pygame.Rect(1180, 180, 115, 56)
        
        # Linha 3 - Botões de Waypoint: GRAVAR | NAVEGAR/REINICIAR | ZERAR (3 botões iguais)
        self.btn_record_wp  = pygame.Rect(600,  250, 230, 42)
        self.btn_return_wp  = pygame.Rect(840,  250, 200, 42)
        self.btn_clear_wps  = pygame.Rect(1050, 250, 210, 42)
        
        # Limite padrão para alerta de desvio lateral (XTK)
        # Inicializado em 0.3 NM por padrão. Será atualizado dinamicamente pelo nav_state.
        self.xtk_tolerance = 0.3

        # Controles VNAV (Painel Direito Extra)
        self.vnav_cal_input_rect = pygame.Rect(1380, 205, 200, 40)
        self.btn_toggle_vnav_cal = pygame.Rect(1600, 205, 110, 40)

        self.vnav_alt_input_rect = pygame.Rect(1380, 290, 200, 40)
        self.btn_set_vnav_alt = pygame.Rect(1600, 290, 110, 40)
        self.vnav_tol_input_rect = pygame.Rect(1380, 375, 200, 40)
        self.btn_set_vnav_tol = pygame.Rect(1600, 375, 110, 40)

        self.btn_cycle_chart = pygame.Rect(1380, 670, 360, 50)
        self.btn_load_chart = pygame.Rect(1380, 740, 360, 60)

        # Botões temporários de seleção de lado do circuito (Card IA) e avanço manual de waypoint
        self.btn_side_left = None
        self.btn_side_right = None
        self.btn_next_wp = None

        # Filtro passa-baixa de firmeza hidráulica da barra do Flight Director (absorve tremor e cambaleio do manche)
        self.smooth_fd_ratio_x = 0.0


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

    def draw_data_box(self, surface, rect, title, value, color=WHITE, border_color=GRAY):
        """Desenha uma caixinha de dados de telemetria (HDG, TRK, SPD, ALT)."""
        pygame.draw.rect(surface, PANEL_BG, rect, border_radius=10)
        pygame.draw.rect(surface, border_color, rect, 2, border_radius=10)
        self.draw_text(surface, title, (rect.centerx, rect.y + 12), self.f_label, WHITE, align="center")
        # Usa fonte compacta automaticamente se o valor tiver 5 ou mais caracteres
        font = self.f_data_compact if len(str(value)) >= 5 else self.f_data
        self.draw_text(surface, value, (rect.centerx, rect.y + 44), font, color, align="center")

    # ========================================================================
    # RENDERIZACAO DO HORIZONTE ARTIFICIAL (CEU AZUL, TERRA MARROM E PITCH LADDER)
    # ========================================================================
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

        # Rotaciona a superfície com base na inclinação das asas (bank real)
        rotated = pygame.transform.rotozoom(world, bank, 1.0)

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

    # ========================================================================
    # INDICADOR DE ROLAMENTO E ARCO DE CURVA (MARCACOES A 10, 20, 30, 45 E 60 GRAUS)
    # ========================================================================
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

    # ========================================================================
    # SIMBOLO DE REFERENCIA DA AERONAVE (CRUZ CENTRAL E ASAS FIXAS NO PFD)
    # ========================================================================
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
        
        # Oculta o FD se o sistema não estiver engajado, nas fases 1 a 4, ou em subida de arremetida
        if not is_active or (1 <= v_phase <= 4) or is_ma_climbing:
            return

        # Para rotas de waypoints manuais (sem procedimento IFR formal),
        # o Flight Director NÃO entra durante as fases de afastamento e perna paralela (fases 1 a 4).
        # Ele ENTRA quando o piloto afasta 1 milha do través e o sistema comanda a curva base (fases 5, 6 e 7)!
        if nav_state.get("navigation_mode") in ("route", "route_finished") and not nav_state.get("active_procedure") and not nav_state.get("gs_synthetic_active"):
            if 1 <= v_phase < 5:
                return

        # Modo GS Sintético: só ativa o Flight Director na interceptação ou final (fase >= 6)
        if nav_state.get("gs_synthetic_active") and not nav_state.get("active_procedure") and v_phase < 6:
            return
            
        # --- LNAV (Barra Vertical) ---
        # A barra vertical atua como um comando de Roll (Rolamento) / Localizer.
        xtk_nm = nav_state.get("xtk", 0.0)
        
        # Na reta final (curta final < 2 NM da cabeceira no GS Sintético),
        # a sensibilidade afunila dinamicamente como um Localizer real (ILS),
        # permitindo que desvios de 20 a 50m sejam claramente visíveis e corrigidos.
        eff_tolerance = self.xtk_tolerance
        if nav_state.get("gs_synthetic_active") and not nav_state.get("active_procedure"):
            dist_thr = nav_state.get("gs_dist_to_threshold")
            if dist_thr is not None and dist_thr < 2.0:
                # Interpola de 0.3 NM (a 2 NM) até 0.06 NM (~110m na cabeceira)
                ratio_dist = max(0.0, dist_thr / 2.0)
                eff_tolerance = 0.06 + (self.xtk_tolerance - 0.06) * ratio_dist
                eff_tolerance = max(0.04, min(self.xtk_tolerance, eff_tolerance))

        # --- LNAV (Barra Vertical) ---
        target_hdg = nav_state.get("target_heading_float", nav_state.get("target_heading"))
        current_hdg = nav_state.get("current_heading")
        use_steering = nav_state.get("fd_steering_mode", True)
        
        if use_steering and target_hdg and current_hdg is not None and v_phase in (0, 5, 6, 7):
            # OPÇÃO 2: Comando Real de Roll Steering com Amortecimento Cinemático Suave
            # Erro entre a proa ordenada pela IA e a proa atual do avião (-180° a +180°)
            hdg_diff = (target_hdg - current_hdg + 180) % 360 - 180
            
            # Amortecimento Cinemático por Taxa de Curva (Turn Rate Lead):
            # tau_lead ajustado para 0.35s (firme e amortecido): evita que mexidas no manche façam a barra cambalear
            turn_rate = nav_state.get("turn_rate_dps", 0.0)
            tau_lead = 0.35  # segundos de antecipação suave
            damped_diff = hdg_diff - (turn_rate * tau_lead)
            
            # Escala angular expandida para 22.0° (elimina a hipersensibilidade do instrumento)
            max_scale_deg = 22.0
            raw_ratio_x = damped_diff / max_scale_deg
        else:
            # OPÇÃO 1 (Modo Clássico): Agulha de distância pura (CDI)
            raw_ratio_x = -xtk_nm / eff_tolerance
        
        # A saturação (clipping) limita a barra às bordas do instrumento se o erro exceder a tolerância.
        raw_ratio_x = max(-1.0, min(1.0, raw_ratio_x))
        
        # Filtro passa-baixa de firmeza hidráulica (Low-Pass Filter a 30 FPS):
        # 82% do valor anterior + 18% do valor novo.
        # Deixa a barra extremamente firme, sólida e estável, absorvendo qualquer tremor ou mexida no manche!
        if not hasattr(self, 'smooth_fd_ratio_x') or self.smooth_fd_ratio_x is None:
            self.smooth_fd_ratio_x = raw_ratio_x
        else:
            self.smooth_fd_ratio_x = self.smooth_fd_ratio_x * 0.82 + raw_ratio_x * 0.18

        x_offset = int(self.smooth_fd_ratio_x * max_x_offset)
        
        # Renderiza a Barra Vertical Magenta
        fd_x = CENTER[0] + x_offset
        pygame.draw.line(surface, MAGENTA, (fd_x, CENTER[1] - 100), (fd_x, CENTER[1] + 100), 3)

        # --- VNAV (Barra Horizontal) ---
        # A barra horizontal atua como um comando de Pitch (Arfagem).
        # Tenta usar a altitude do Glidepath (se existir), senão usa a estática
        vnav_gs = nav_state.get("vnav_gs_alt")
        target_alt = vnav_gs if vnav_gs is not None else nav_state.get("vnav_target_alt")
        if target_alt is not None:
            vnav_tolerance = max(10.0, float(nav_state.get("vnav_tolerance", 200) or 200))
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
        self.waypoint_scale_rects = []

        for i in range(MAX_WPS):
            # Desenha de cima para baixo
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
                elif nav_mode == "route_finished":
                    color, radius, border = GREEN, 7, 0
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





    def draw_brand_logo(self, surface):
        """
        Desenha a marca oficial NAVEG-AI abaixo do horizonte artificial:
        - Perfeitamente alinhada no eixo central do horizonte (X=280);
        - Texto em branco em destaque;
        - Moldura retangular vermelha chamativa em volta.
        """
        center_x = 280
        center_y = 715
        box_w = 280
        box_h = 74
        logo_rect = pygame.Rect(center_x - box_w // 2, center_y - box_h // 2, box_w, box_h)

        # Fundo escuro do cockpit + moldura vermelha chamativa em volta
        pygame.draw.rect(surface, (35, 12, 16), logo_rect, border_radius=12)
        pygame.draw.rect(surface, (225, 30, 30), logo_rect, 3, border_radius=12)

        # Texto NAVEG-AI em branco puro
        self.draw_text(surface, "NAVEG-AI", logo_rect.center, self.f_logo, WHITE, align="center")

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
        # Barra Superior de Status Aviônica (GPS e IA Co-Piloto) - Sem sobreposição
        status_bar_rect = pygame.Rect(560, 2, 756, 34)
        pygame.draw.rect(surface, (14, 17, 24), status_bar_rect, border_radius=6)
        pygame.draw.rect(surface, (55, 68, 90), status_bar_rect, 1, border_radius=6)

        # 1. Indicador de Satélites GPS (Lado Esquerdo da Barra Superior)
        sats = data.get("satellites", 0)
        qual = data.get("gps_quality", 0)
        if sats >= 6:
            sat_color = GREEN
            sat_txt = f"SATS: {sats} (FIX 3D OK)"
        elif sats >= 4:
            sat_color = YELLOW
            sat_txt = f"SATS: {sats} (FIX 2D)"
        elif sats > 0 or qual > 0:
            sat_color = (255, 165, 0)
            sat_txt = f"SATS: {sats} (ADQUIRINDO)"
        elif data.get("connected") and not data.get("waiting_for_flight"):
            sat_color = (255, 110, 110)
            sat_txt = "SATS: BUSCANDO SINAL..."
        else:
            sat_color = GRAY
            sat_txt = "SATS: DESCONECTADO"

        pygame.draw.circle(surface, sat_color, (status_bar_rect.x + 18, status_bar_rect.centery), 6)
        sat_surf = self.f_status.render(sat_txt, True, sat_color)
        surface.blit(sat_surf, (status_bar_rect.x + 32, status_bar_rect.centery - sat_surf.get_height() // 2))

        # 2. Indicador de Status da IA Co-Piloto (Lado Direito da Barra Superior)
        is_ai_online = nav_state.get("ai_online", False)
        if is_ai_online:
            ai_status_color = AI_PURPLE
            ai_status_txt = "IA CO-PILOTO: ONLINE (Qwen 2.5)"
        else:
            ai_status_color = (120, 160, 220)
            ai_status_txt = "IA CO-PILOTO: STANDBY / BACKUP"

        ai_surf = self.f_status.render(ai_status_txt, True, ai_status_color)
        ai_x = status_bar_rect.right - ai_surf.get_width() - 18
        pygame.draw.circle(surface, ai_status_color, (ai_x - 14, status_bar_rect.centery), 6)
        surface.blit(ai_surf, (ai_x, status_bar_rect.centery - ai_surf.get_height() // 2))

        panel_rect = pygame.Rect(560, 38, 756, 838)
        pygame.draw.rect(surface, PANEL_BG, panel_rect, border_radius=14)
        pygame.draw.rect(surface, GRAY, panel_rect, 2, border_radius=14)

        # Título do Painel (Isolado e centralizado sem colisões)
        self.draw_text(surface, "SISTEMA DE NAVEGAÇÃO LATERAL", (panel_rect.centerx, 58), self.f_title, AI_PURPLE, align="center")

        is_active = nav_state["active"]
        xtk = nav_state["xtk"]
        course = nav_state["course"]
        target_hdg = nav_state["target_heading"]
        waypoints = nav_state.get("recorded_waypoints", [])
        nav_mode = nav_state.get("navigation_mode", "manual")
        active_wp_idx = nav_state.get("active_waypoint_index", 0)
        
        # Sincroniza a tolerância interna do PFD com o valor configurado no nav_state
        self.xtk_tolerance = nav_state.get("xtk_tolerance", 0.3)

        # Indicador de Status Visual (ATIVO / STANDBY) - Posicionado limpo acima do botão Monitor
        status_color = GREEN if is_active else GRAY
        status_text = "ATIVO" if is_active else "STANDBY"
        pygame.draw.circle(surface, status_color, (615, 84), 6)
        self.draw_text(surface, f"STATUS: {status_text}", (630, 76), self.f_label, status_color)

        # ── LINHA 1: Botão Monitor ──────────────────────────
        system_on = nav_state.get("system_on", False)
        btn_monitor_color = RED if system_on else GREEN
        if self.btn_monitor.collidepoint(mouse_pos):
            btn_monitor_color = (max(0, btn_monitor_color[0]-40), max(0, btn_monitor_color[1]-40), max(0, btn_monitor_color[2]-40))
        pygame.draw.rect(surface, btn_monitor_color, self.btn_monitor, border_radius=10)
        pygame.draw.rect(surface, WHITE, self.btn_monitor, 2, border_radius=10)
        btn_label = "PARAR MONITOR" if system_on else "INICIAR MONITOR"
        self.draw_text(surface, btn_label, self.btn_monitor.center, self.f_btn, BLACK if not system_on else WHITE, align="center")

        # ── LINHA 1: Botão LOG (gravação de telemetria CSV) ──
        log_mode = nav_state.get("log_mode", False)
        is_logging = nav_state.get("csv_logging_active", False)
        if is_logging:
            btn_log_color = (215, 35, 35)
        elif log_mode:
            btn_log_color = (255, 140, 0)
        else:
            btn_log_color = GRAY

        if self.btn_log.collidepoint(mouse_pos):
            btn_log_color = (max(0, btn_log_color[0]-35), max(0, btn_log_color[1]-35), max(0, btn_log_color[2]-35))
        pygame.draw.rect(surface, btn_log_color, self.btn_log, border_radius=10)
        pygame.draw.rect(surface, WHITE, self.btn_log, 2, border_radius=10)

        if is_logging:
            blink = (pygame.time.get_ticks() // 400) % 2 == 0
            if blink:
                pygame.draw.circle(surface, WHITE, (self.btn_log.x + 18, self.btn_log.centery), 5)
            self.draw_text(surface, "REC", (self.btn_log.x + 48, self.btn_log.centery), self.f_btn, WHITE, align="center")
        else:
            self.draw_text(surface, "LOG", self.btn_log.center, self.f_btn, BLACK if log_mode else WHITE, align="center")

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
        self.draw_text(surface, "SET", self.btn_set_course.center, self.f_btn, WHITE, align="center")

        # ── LINHA 2: Tolerância XTK (compacta) ──────────────────────────────
        is_tolerance_active = (active_input == "tolerance")
        tolerance_border = BLUE if is_tolerance_active else GRAY
        pygame.draw.rect(surface, BOX_BG, self.tolerance_input_rect, border_radius=10)
        pygame.draw.rect(surface, tolerance_border, self.tolerance_input_rect, 2, border_radius=10)
        self.draw_text(surface, "XTK(NM):", (self.tolerance_input_rect.x, self.tolerance_input_rect.y - 25), self.f_field_label, WHITE)
        
        display_tolerance = tolerance_text + ("|" if is_tolerance_active and (pygame.time.get_ticks() // 400) % 2 == 0 else "")
        if not tolerance_text and not is_tolerance_active:
            self.draw_text(surface, "0.3", (self.tolerance_input_rect.x + 8, self.tolerance_input_rect.y + 10), self.f_data_compact, WHITE)
        else:
            self.draw_text(surface, display_tolerance, (self.tolerance_input_rect.x + 8, self.tolerance_input_rect.y + 10), self.f_data_compact, WHITE)

        is_xtk_on = nav_state.get("xtk_custom_set", False)
        btn_tolerance_base = BTN_ON_GREEN if is_xtk_on else (80, 80, 80)
        btn_tolerance_color = (min(255, btn_tolerance_base[0]+25), min(255, btn_tolerance_base[1]+25), min(255, btn_tolerance_base[2]+25)) \
                              if self.btn_set_tolerance.collidepoint(mouse_pos) else btn_tolerance_base
        pygame.draw.rect(surface, btn_tolerance_color, self.btn_set_tolerance, border_radius=10)
        pygame.draw.rect(surface, WHITE, self.btn_set_tolerance, 1, border_radius=10)
        btn_tol_label = "ON" if is_xtk_on else "OFF"
        self.draw_text(surface, btn_tol_label, self.btn_set_tolerance.center, self.f_btn, WHITE, align="center")

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

        # Botão LIMPAR RAMPA (Linha 1: só aparece se houver rampa salva)
        saved_gs = nav_state.get("saved_gs_approach")
        if saved_gs:
            self.btn_clear_gs = pygame.Rect(1055, 100, 115, 54)
            clear_gs_color = (130, 30, 30)
            if self.btn_clear_gs.collidepoint(mouse_pos):
                clear_gs_color = (170, 40, 40)
            pygame.draw.rect(surface, clear_gs_color, self.btn_clear_gs, border_radius=10)
            pygame.draw.rect(surface, WHITE, self.btn_clear_gs, 2, border_radius=10)
            self.draw_text(surface, "LIMPAR", (self.btn_clear_gs.centerx, self.btn_clear_gs.y + 16), self.f_btn_sub, WHITE, align="center")
            self.draw_text(surface, "RAMPA", (self.btn_clear_gs.centerx, self.btn_clear_gs.y + 38), self.f_btn, WHITE, align="center")
        else:
            self.btn_clear_gs = None

        # Botão SALVAR / CARREGAR RAMPA (Linha 1: só aparece se houver rampa salva ou se o 50FT estiver ativo)
        if saved_gs or gs_active:
            self.btn_save_gs = pygame.Rect(1180, 100, 115, 54)
            btn_save_text1 = "CARREGAR" if saved_gs else "SALVAR"
            btn_save_text2 = "RAMPA"
            save_btn_color = (0, 150, 100) if saved_gs else (80, 80, 80)
            if self.btn_save_gs.collidepoint(mouse_pos):
                save_btn_color = (max(0, save_btn_color[0]-20), max(0, save_btn_color[1]-20), max(0, save_btn_color[2]-20))
            pygame.draw.rect(surface, save_btn_color, self.btn_save_gs, border_radius=10)
            pygame.draw.rect(surface, WHITE, self.btn_save_gs, 2, border_radius=10)
            self.draw_text(surface, btn_save_text1, (self.btn_save_gs.centerx, self.btn_save_gs.y + 16), self.f_btn_sub, WHITE, align="center")
            self.draw_text(surface, btn_save_text2, (self.btn_save_gs.centerx, self.btn_save_gs.y + 38), self.f_btn, WHITE, align="center")
        else:
            self.btn_save_gs = None

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
        self.draw_text(surface, btn_gs_line1, (self.btn_start_gs.centerx, self.btn_start_gs.y + 17), self.f_btn_sub, gs_txt_color, align="center")
        self.draw_text(surface, btn_gs_line2, (self.btn_start_gs.centerx, self.btn_start_gs.y + 39), self.f_btn_big, gs_txt_color, align="center")

        # ── LINHA 3: Botões de Waypoint ───────────────────────────────────────
        wp_count = len(waypoints)
        max_wps = 10

        # Botão GRAVAR WAYPOINT
        display_count = min(wp_count, max_wps)
        record_label = f"GRAVAR WYP ({display_count}/{max_wps})"
        has_gps_fix = data.get("gps_fix", True)
        if data.get("is_real_pi", False) or (data.get("connected") and data.get("satellites", 99) < 4 and not data.get("waiting_for_flight")):
            has_gps_fix = False

        if wp_count >= max_wps:
            rec_base = GRAY
            rec_border = GRAY
            rec_txt_color = WHITE
        elif not has_gps_fix:
            rec_base = (70, 50, 30)
            rec_border = (255, 140, 40)
            rec_txt_color = (255, 200, 120)
        else:
            rec_base = BLUE
            rec_border = WHITE
            rec_txt_color = WHITE

        rec_color = (max(0, rec_base[0]-40), max(0, rec_base[1]-40), max(0, rec_base[2]-40)) \
                    if self.btn_record_wp.collidepoint(mouse_pos) else rec_base
        pygame.draw.rect(surface, rec_color, self.btn_record_wp, border_radius=10)
        pygame.draw.rect(surface, rec_border, self.btn_record_wp, 1, border_radius=10)
        self.draw_text(surface, record_label, self.btn_record_wp.center, self.f_btn, rec_txt_color, align="center")

        # Botão RETORNAR/REINICIAR WAYPOINT
        is_route_finished = (nav_mode == "route_finished")
        is_route_active = (nav_mode == "route")
        is_navigating = (is_route_active or is_route_finished)
        v_phase = nav_state.get("vectoring_phase", 0) if nav_state else 0

        if is_route_finished:
            ret_label = "RETORNAR WP01"
            ret_base_color = MAGENTA
        elif is_route_active:
            ret_label = "RETORNAR WP01" if v_phase == 0 else "REINICIAR WYP"
            ret_base_color = BLUE
        else:
            ret_label = "NAVEGAR WYP"
            ret_base_color = MAGENTA if wp_count > 0 else GRAY
        
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
        # Reset botões contextuais por padrão a cada frame
        self.btn_side_left = None
        self.btn_side_right = None
        self.btn_next_wp = None

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
            log_on = nav_state.get("log_final_active", False)  # Modo LOG ativo e na final
            
            # Curso travado e desvio XTK
            side = ""
            xtk_color = GREEN
            # Só mostra R ou L se não estiver nas fases de vetoramento inicial (1 a 5)
            if abs(xtk) >= 0.005 and not (1 <= v_phase <= 5):
                side = " R" if xtk > 0 else " L"
            if abs(xtk) > self.xtk_tolerance:
                xtk_color = RED

            # Label de waypoint ativo (modo rota) ou curso manual (modo manual)
            dist_to_wp = nav_state.get("dist_to_wp")
            if 1 <= v_phase <= 5:
                # Durante vetoramento/circuito (1 a 5): exibe apenas ALVO e a única distância relevante (Través ou Perna do Vento)
                if nav_mode == "route" and waypoints and 0 <= active_wp_idx < len(waypoints):
                    wp_name = waypoints[active_wp_idx].get("name", f"WP{active_wp_idx+1}")
                    self.draw_text(surface, "ALVO", (600, 325), self.f_label, WHITE)
                    self.draw_text(surface, wp_name, (600, 347), self.f_data, MAGENTA)
                else:
                    self.draw_text(surface, "CURSO TRAVADO", (600, 325), self.f_label, WHITE)
                    self.draw_text(surface, f"{int(course):03}°", (600, 347), self.f_data, YELLOW)

                long_val = nav_state.get("longitudinal_from_origin")
                is_takeoff = bool(nav_state.get("gs_takeoff_active", False) and nav_state.get("gs_synthetic_active", False))
                is_go_around = nav_state.get("is_go_around", False)
                abeam_passed = nav_state.get("abeam_passed", False)
                if is_go_around:
                    val_disp = 0.0 if long_val is None else abs(long_val)
                    self.draw_text(surface, "ARREMETIDA / SUBIDA", (760, 325), self.f_label, WHITE)
                    self.draw_text(surface, f"{val_disp:.1f} NM", (760, 347), self.f_data, GREEN)
                elif is_takeoff:
                    val_disp = 0.0 if long_val is None else abs(long_val)
                    self.draw_text(surface, "DECOLAGEM / SUBIDA", (760, 325), self.f_label, WHITE)
                    self.draw_text(surface, f"{val_disp:.1f} NM", (760, 347), self.f_data, YELLOW)
                elif v_phase == 5:
                    self.draw_text(surface, "PERNA BASE", (760, 325), self.f_label, WHITE)
                    self.draw_text(surface, f"{abs(xtk):.1f} NM", (760, 347), self.f_data, MAGENTA)
                elif long_val is not None:
                    if not abeam_passed and long_val > 0.0:
                        val_disp = 0.0 if abs(long_val) < 0.05 else abs(long_val)
                        self.draw_text(surface, "DISTÂNCIA TRAVÉS", (760, 325), self.f_label, WHITE)
                        self.draw_text(surface, f"{val_disp:.1f} NM", (760, 347), self.f_data, YELLOW)
                    else:
                        is_gs = nav_state.get("gs_synthetic_active", False)
                        max_limit = 10.0 if is_gs else 1.0
                        disp_val = min(max_limit, abs(long_val))
                        target_dist_str = "10 NM" if is_gs else "1.0 NM"
                        self.draw_text(surface, "PERNA DO VENTO", (760, 325), self.f_label, WHITE)
                        if is_gs and disp_val >= 10.0:
                            self.draw_text(surface, "10 / 10 NM", (760, 347), self.f_data, GREEN)
                        elif not is_gs and disp_val >= 1.0:
                            self.draw_text(surface, "1.0 / 1.0 NM", (760, 347), self.f_data, GREEN)
                        else:
                            self.draw_text(surface, f"{disp_val:.1f} / {target_dist_str}", (760, 347), self.f_data, GREEN)
            else:
                # Fora do circuito (voando waypoints normais, interceptação ou final)
                if nav_mode == "route" and waypoints and 0 <= active_wp_idx < len(waypoints):
                    wp_name = waypoints[active_wp_idx].get("name", f"WP{active_wp_idx+1}")
                    self.draw_text(surface, "ALVO", (600, 325), self.f_label, WHITE)
                    self.draw_text(surface, wp_name, (600, 347), self.f_data, MAGENTA)
                    
                    if dist_to_wp is not None:
                        self.draw_text(surface, "DISTÂNCIA", (740, 325), self.f_label, WHITE)
                        self.draw_text(surface, f"{dist_to_wp:.1f} NM", (740, 347), self.f_data, GREEN)
                else:
                    self.draw_text(surface, "CURSO TRAVADO", (600, 325), self.f_label, WHITE)
                    self.draw_text(surface, f"{int(course):03}°", (600, 347), self.f_data, YELLOW)

                # Desvio Lateral XTK com indicação de lado (L/R)
                self.draw_text(surface, "DESVIO LATERAL (XTK)", (920, 325), self.f_label, WHITE)
                self.draw_text(surface, f"{abs(xtk):.3f} NM{side}", (920, 347), self.f_data, xtk_color)

            # ── Painel da IA ──────────────────────────────────────────────────
            ai_card_y = 390
            ai_card_rect = pygame.Rect(590, ai_card_y, 700, 330)
            
            v_phase = nav_state.get("vectoring_phase", 0)
            
            # Alerta visual (borda vermelha): aciona quando o desvio lateral XTK excede a tolerância (ex: 0.3 NM)
            card_alert = (abs(xtk) > self.xtk_tolerance) if (v_phase == 0 or v_phase >= 7) else False
            if card_alert:
                pygame.draw.rect(surface, (70, 15, 15), ai_card_rect, border_radius=15)
                pygame.draw.rect(surface, RED, ai_card_rect, 3, border_radius=15)
            else:
                is_ai_active = nav_state.get("ai_online", False)
                card_border = AI_PURPLE if is_ai_active else (80, 130, 220)
                pygame.draw.rect(surface, (25, 20, 45), ai_card_rect, border_radius=15)
                pygame.draw.rect(surface, card_border, ai_card_rect, 2, border_radius=15)

            is_ai_active = nav_state.get("ai_online", False)
            title_text = "CO-PILOTO NEURAL (IA: ONLINE)" if is_ai_active else "SISTEMA DE VETORAÇÃO (BACKUP)"
            title_color = AI_PURPLE if is_ai_active else (100, 160, 255)
            pygame.draw.circle(surface, title_color, (630, ai_card_y + 30), 8)
            self.draw_text(surface, title_text, (645, ai_card_y + 22), self.f_btn, title_color)

            # NavMath já importado e disponível globalmente
            heading_error = NavMath.normalize_angle(data["track"] - course)
            is_aligned = abs(heading_error) <= 5.0

            # ── Alerta de Overshoot (Padrão Internacional: pisca por 10s e depois desaparece) ──
            ov_time = nav_state.get("overshoot_time", 0)
            ov_elapsed = time.time() - ov_time if ov_time else 999.0
            overshoot_active = bool(nav_state.get("overshoot_banner") and (ov_elapsed < 10.0))
            is_real_overshoot = bool(overshoot_active and nav_state.get("overshoot_from") and nav_state.get("overshoot_to"))

            if is_real_overshoot:
                # Pisca ritmicamente a cada ~350ms durante os 10 segundos apenas quando houve overshoot real de fixo
                blink_ov = (pygame.time.get_ticks() // 350) % 2 == 0
                if blink_ov:
                    self.draw_text(surface, "ALERTA: FIXO ULTRAPASSADO (OVERSHOOT):", (620, ai_card_y + 58), self.f_label, YELLOW)
                    ov_from = nav_state.get("overshoot_from", "WP")
                    ov_to = nav_state.get("overshoot_to", "WP")
                    self.draw_text(surface, f"{ov_from} -> AVANÇANDO PARA {ov_to}", (620, ai_card_y + 92), self.f_data_compact, YELLOW)
            else:
                self.draw_text(surface, "PROA DE INTERCEPTAÇÃO SUGERIDA:", (620, ai_card_y + 58), self.f_label, WHITE)
                
                if nav_mode == "route_finished":
                    self.draw_text(surface, "---", (620, ai_card_y + 92), self.f_data, GREEN)
                elif v_phase > 0 and v_phase < 7:
                    # Durante o vetoramento (fases 1 a 6), a tela de circuito tem prioridade total, mesmo com LOG ativo
                    c_side = nav_state.get("circuit_side")
                    gs_active = nav_state.get("gs_synthetic_active", False)
                    pat_course = nav_state.get("pattern_course")
                    if pat_course is None:
                        pat_course = course if course is not None else (target_hdg or 0.0)
                    recip = int(round((pat_course + 180) % 360)) % 360
                    
                    if not c_side:
                        if nav_state.get("is_return_approach", False):
                            # Retorno da Rampa (CARREGAR RAMPA após waypoints): botões de escolha com Direct-To (>4 NM) ou Aproximação (<=4 NM)
                            d_active = nav_state.get("direct_to_airport", False)
                            d_hdg = nav_state.get("direct_to_heading", target_hdg)
                            d_dist = nav_state.get("direct_to_dist", nav_state.get("gs_dist_to_threshold", 0.0) or 0.0)

                            if d_active:
                                nav_state["_appr_4nm_start"] = None
                                self.draw_text(surface, f"DIRETO CABECEIRA -> {d_hdg:03}° ({d_dist:.1f} NM)", (620, ai_card_y + 92), self.f_data_compact, YELLOW)
                            else:
                                # Entrou nas 4 milhas (Aproximação): pisca por 10 segundos e depois fica estático
                                now_t = time.time()
                                appr_start = nav_state.get("_appr_4nm_start")
                                if appr_start is None:
                                    nav_state["_appr_4nm_start"] = now_t
                                    appr_start = now_t
                                
                                elapsed_appr = now_t - appr_start
                                draw_appr = True
                                if elapsed_appr < 10.0:
                                    draw_appr = (pygame.time.get_ticks() // 350) % 2 == 0
                                
                                if draw_appr:
                                    self.draw_text(surface, f"APROXIMAÇÃO ({d_dist:.1f} NM) - AGUARDE CIRCUITO", (620, ai_card_y + 92), self.f_data_compact, YELLOW)

                            self.btn_side_left = pygame.Rect(620, ai_card_y + 126, 220, 34)
                            self.btn_side_right = pygame.Rect(860, ai_card_y + 126, 220, 34)
                            
                            btn_l_color = (15, 45, 95) if self.btn_side_left.collidepoint(mouse_pos) else (20, 50, 110)
                            btn_r_color = (15, 45, 95) if self.btn_side_right.collidepoint(mouse_pos) else (20, 50, 110)
                            
                            pygame.draw.rect(surface, btn_l_color, self.btn_side_left, border_radius=8)
                            pygame.draw.rect(surface, (80, 160, 255), self.btn_side_left, 2, border_radius=8)
                            self.draw_text(surface, "CIRCUITO ESQUERDA", self.btn_side_left.center, self.f_field_label, WHITE, align="center")
                            
                            pygame.draw.rect(surface, btn_r_color, self.btn_side_right, border_radius=8)
                            pygame.draw.rect(surface, (80, 160, 255), self.btn_side_right, 2, border_radius=8)
                            self.draw_text(surface, "CIRCUITO DIREITA", self.btn_side_right.center, self.f_field_label, WHITE, align="center")
                        else:
                            # Decolagem inicial no solo/subida ou Arremetida:
                            if nav_state.get("is_go_around", False):
                                self.draw_text(surface, f"ARREMETIDA / SUBIDA (PROA {int(course):03}°)", (620, ai_card_y + 92), self.f_data_compact, GREEN)
                            elif nav_state.get("gs_takeoff_active", False) and nav_state.get("gs_synthetic_active", False):
                                self.draw_text(surface, f"DECOLAGEM / SUBIDA (PROA {int(course):03}°)", (620, ai_card_y + 92), self.f_data_compact, GREEN)
                            else:
                                self.draw_text(surface, "IDENTIFICANDO CIRCUITO...", (620, ai_card_y + 92), self.f_data_compact, YELLOW)
                            
                            self.btn_side_left = pygame.Rect(620, ai_card_y + 130, 220, 32)
                            self.btn_side_right = pygame.Rect(860, ai_card_y + 130, 220, 32)
                            
                            btn_l_color = (15, 45, 95) if self.btn_side_left.collidepoint(mouse_pos) else (20, 50, 110)
                            btn_r_color = (15, 45, 95) if self.btn_side_right.collidepoint(mouse_pos) else (20, 50, 110)
                            
                            pygame.draw.rect(surface, btn_l_color, self.btn_side_left, border_radius=8)
                            pygame.draw.rect(surface, (80, 160, 255), self.btn_side_left, 2, border_radius=8)
                            self.draw_text(surface, "CIRCUITO ESQUERDA", self.btn_side_left.center, self.f_field_label, WHITE, align="center")
                            
                            pygame.draw.rect(surface, btn_r_color, self.btn_side_right, border_radius=8)
                            pygame.draw.rect(surface, (80, 160, 255), self.btn_side_right, 2, border_radius=8)
                            self.draw_text(surface, "CIRCUITO DIREITA", self.btn_side_right.center, self.f_field_label, WHITE, align="center")
                    else:
                        lado_curva = "DIREITA" if c_side == "R" else "ESQUERDA"
                        base_hdg = (pat_course + 90) % 360 if c_side == "L" else (pat_course - 90 + 360) % 360
                        raw_fc = nav_state.get("gs_inbound_course") if nav_state.get("gs_synthetic_active") else nav_state.get("course", 0.0)
                        final_crs = int(round(raw_fc if raw_fc is not None else (nav_state.get("course") or 0.0))) % 360

                        if v_phase in (1, 2, 3):
                            if v_phase == 1:
                                self.draw_text(surface, f"CURVA {lado_curva} -> PARALELA ({recip:03}°)", (620, ai_card_y + 92), self.f_data_compact, YELLOW)
                            else:
                                self.draw_text(surface, f"PERNA PARALELA -> {recip:03}° ({lado_curva})", (620, ai_card_y + 92), self.f_data_compact, YELLOW)
                            self.draw_text(surface, f"BASE ANTECIPADA -> {int(base_hdg):03}° ({lado_curva})", (620, ai_card_y + 126), self.f_btn, BLUE)

                            # Botões de alternância direta (para o piloto poder mudar se quiser)
                            self.btn_side_left = pygame.Rect(620, ai_card_y + 158, 220, 30)
                            self.btn_side_right = pygame.Rect(860, ai_card_y + 158, 220, 30)
                            
                            is_l = (c_side == "L")
                            btn_l_bg = (10, 60, 30) if is_l else ((15, 45, 95) if self.btn_side_left.collidepoint(mouse_pos) else (20, 50, 110))
                            btn_l_border = GREEN if is_l else (80, 160, 255)
                            btn_l_txt = "CIRCUITO ESQ (ATIVO)" if is_l else "CIRCUITO ESQUERDA"
                            
                            is_r = (c_side == "R")
                            btn_r_bg = (10, 60, 30) if is_r else ((15, 45, 95) if self.btn_side_right.collidepoint(mouse_pos) else (20, 50, 110))
                            btn_r_border = GREEN if is_r else (80, 160, 255)
                            btn_r_txt = "CIRCUITO DIR (ATIVO)" if is_r else "CIRCUITO DIREITA"

                            pygame.draw.rect(surface, btn_l_bg, self.btn_side_left, border_radius=8)
                            pygame.draw.rect(surface, btn_l_border, self.btn_side_left, 2, border_radius=8)
                            self.draw_text(surface, btn_l_txt, self.btn_side_left.center, self.f_field_label, WHITE, align="center")

                            pygame.draw.rect(surface, btn_r_bg, self.btn_side_right, border_radius=8)
                            pygame.draw.rect(surface, btn_r_border, self.btn_side_right, 2, border_radius=8)
                            self.draw_text(surface, btn_r_txt, self.btn_side_right.center, self.f_field_label, WHITE, align="center")

                        elif v_phase in (4, 5):
                            self.btn_side_left = None
                            self.btn_side_right = None
                            self.draw_text(surface, f"BASE {int(base_hdg):03}° -> FINAL {final_crs:03}° ({lado_curva})", (620, ai_card_y + 92), self.f_data_compact, MAGENTA)
                        elif v_phase == 6:
                            self.btn_side_left = None
                            self.btn_side_right = None
                            cur_h = data.get("heading", data.get("track", target_hdg))
                            diff_h = (target_hdg - cur_h + 180.0) % 360.0 - 180.0
                            
                            # Histerese de Estado (Schmitt Trigger): elimina alternância rápida de telas
                            was_on_centerline = nav_state.get("_p6_on_centerline", False)
                            was_turning = nav_state.get("_p6_turning", False)
                            turn_dir = nav_state.get("_p6_turn_dir", "L")
                            
                            # 1. Histerese do Corredor Central (entra com <= 0.020 NM, só sai se afastar > 0.035 NM):
                            if was_on_centerline:
                                is_on_centerline = (abs(xtk) <= 0.035 and abs(heading_error) <= 6.0)
                            else:
                                is_on_centerline = (abs(xtk) <= 0.020 and abs(heading_error) <= 5.0)
                            nav_state["_p6_on_centerline"] = is_on_centerline
                            
                            if is_on_centerline:
                                nav_state["_p6_turning"] = False
                                self.draw_text(surface, f"MANTER CURSO: {int(course):03}°", (620, ai_card_y + 92), self.f_data, GREEN)
                            else:
                                # 2. Histerese de Curva vs Manter (entra com >= 5.0°, só sai ao alinhar a <= 2.5°):
                                if was_turning:
                                    is_turning = (abs(diff_h) > 2.5)
                                    if abs(diff_h) >= 5.0:
                                        turn_dir = "DIR" if diff_h > 0 else "ESQ"
                                else:
                                    is_turning = (abs(diff_h) >= 5.0)
                                    if is_turning:
                                        turn_dir = "DIR" if diff_h > 0 else "ESQ"
                                
                                nav_state["_p6_turning"] = is_turning
                                nav_state["_p6_turn_dir"] = turn_dir
                                
                                if is_turning:
                                    self.draw_text(surface, f"CURVA {turn_dir} -> {int(target_hdg):03}°", (620, ai_card_y + 92), self.f_data, MAGENTA)
                                else:
                                    self.draw_text(surface, f"MANTER {int(target_hdg):03}° INTERCEPTANDO", (620, ai_card_y + 92), self.f_data_compact, BLUE)
                elif log_on:
                    # MODO LOG: atua somente na final (fase 7) ou perna reta (fase 0), desconsiderando a margem de 0.3 NM
                    if nav_state.get("log_correcting"):
                        lado_curva = "ESQ" if nav_state.get("log_turn") == "L" else "DIR"
                        self.draw_text(surface, f"CURVA {lado_curva} -> {nav_state.get('log_target_course', 0):03}°", (620, ai_card_y + 92), self.f_data, MAGENTA)
                    else:
                        if nav_state.get("log_intercepting", False):
                            # Deslocado do eixo: mantém a proa de interceptação até o FD centralizar
                            self.draw_text(surface, f"MANTER {nav_state.get('log_target_course', 0):03}° INTERCEPTANDO", (620, ai_card_y + 92), self.f_data_compact, BLUE)
                        else:
                            self.draw_text(surface, f"MANTER CURSO: {int(course):03}°", (620, ai_card_y + 92), self.f_data, GREEN)
                elif v_phase == 7:
                    # Na fase 7 (sem LOG ativado):
                    was_on_centerline = nav_state.get("_p7_on_centerline", False)
                    was_turning = nav_state.get("_p7_turning", False)
                    turn_dir = nav_state.get("_p7_turn_dir", "L")
                    
                    if was_on_centerline:
                        is_on_centerline = (abs(xtk) <= 0.035 and is_aligned)
                    else:
                        is_on_centerline = (abs(xtk) <= 0.020 and is_aligned)
                    nav_state["_p7_on_centerline"] = is_on_centerline
                    
                    if is_on_centerline:
                        nav_state["_p7_turning"] = False
                        self.draw_text(surface, f"MANTER CURSO: {int(course):03}°", (620, ai_card_y + 92), self.f_data, GREEN)
                    else:
                        cur_h = data.get("heading", data.get("track", target_hdg))
                        diff_h = (target_hdg - cur_h + 180.0) % 360.0 - 180.0
                        if was_turning:
                            is_turning = (abs(diff_h) > 2.5)
                            if abs(diff_h) >= 5.0:
                                turn_dir = "DIR" if diff_h > 0 else "ESQ"
                        else:
                            is_turning = (abs(diff_h) >= 5.0)
                            if is_turning:
                                turn_dir = "DIR" if diff_h > 0 else "ESQ"
                        
                        nav_state["_p7_turning"] = is_turning
                        nav_state["_p7_turn_dir"] = turn_dir
                        
                        if is_turning:
                            self.draw_text(surface, f"CURVA {turn_dir} -> {int(target_hdg):03}°", (620, ai_card_y + 92), self.f_data, MAGENTA)
                        else:
                            self.draw_text(surface, f"MANTER {int(target_hdg):03}° INTERCEPTANDO", (620, ai_card_y + 92), self.f_data_compact, BLUE)
                elif abs(xtk) > self.xtk_tolerance or not is_aligned:
                    self.draw_text(surface, f"{target_hdg:03}°", (620, ai_card_y + 92), self.f_data, MAGENTA)
                else:
                    # Ao invés de ---, mostra o curso desejado grande em verde para o piloto manter
                    self.draw_text(surface, f"MANTER CURSO: {int(course):03}°", (620, ai_card_y + 92), self.f_data, GREEN)

            # Texto explicativo da IA + distância à cabeceira (GS Sintético)
            explanation = ""
            gs_active = nav_state.get("gs_synthetic_active", False)
            gs_dist = nav_state.get("gs_dist_to_threshold")
            
            if overshoot_active:
                explanation = nav_state["overshoot_banner"]
            elif nav_mode == "route_finished":
                explanation = "IA: Último waypoint finalizado. Sem novos waypoints para monitorar. Rota concluída. Aguardando o recomeço da tarefa."
            elif v_phase > 0 and v_phase < 7:
                c_side = nav_state.get("circuit_side")
                lado = "DIREITA" if c_side == "R" else "ESQUERDA"
                pat_course = nav_state.get("pattern_course")
                if pat_course is None:
                    pat_course = course if course is not None else (target_hdg or 0.0)
                recip = int(round((pat_course + 180) % 360)) % 360
                base_hdg = (pat_course + 90) % 360 if c_side == "L" else (pat_course - 90 + 360) % 360
                raw_fc = nav_state.get("gs_inbound_course") if nav_state.get("gs_synthetic_active") else nav_state.get("course", 0.0)
                final_crs = int(round(raw_fc if raw_fc is not None else (nav_state.get("course") or 0.0))) % 360

                if not c_side:
                    if nav_state.get("is_return_approach", False):
                        if gs_active and nav_state.get("direct_to_airport"):
                            d_hdg = nav_state.get("direct_to_heading", target_hdg)
                            d_dist = nav_state.get("direct_to_dist", nav_state.get("gs_dist_to_threshold", 0.0) or 0.0)
                            explanation = f"SISTEMA: Voando direto para o aeródromo (proa {d_hdg:03}°, {d_dist:.1f} NM). Selecione o circuito nos botões acima ou curve para a paralela ({recip:03}°)."
                        elif gs_active:
                            d_dist = nav_state.get("gs_dist_to_threshold", 0.0) or 0.0
                            explanation = f"SISTEMA: A {d_dist:.1f} NM do campo. Aguardando seleção manual de circuito nos botões acima."
                        else:
                            explanation = "SISTEMA: Aguardando primeira curva (direita ou esquerda) para estabelecer na perna paralela."
                    elif nav_state.get("is_go_around", False):
                        explanation = f"SISTEMA: Arremetida em andamento. Suba mantendo a proa da pista ({int(course):03}°) até a altitude de circuito e curve para o tráfego."
                    elif nav_state.get("gs_takeoff_active", False) and gs_active:
                        explanation = "SISTEMA: Rampa 50FT armada. Decole e curve para o circuito, ou selecione o lado nos botões acima."
                    else:
                        explanation = "SISTEMA: Aguardando primeira curva (direita ou esquerda) para estabelecer na perna paralela."
                elif v_phase == 1:
                    explanation = f"SISTEMA: Circuito pela {lado} identificado. Curve para estabelecer na perna paralela ({recip:03}°)."
                elif v_phase == 2:
                    explanation = f"SISTEMA: Perna paralela ({lado}). Mantenha proa {recip:03}° até cruzar o través."
                elif v_phase == 3:
                    explanation = f"SISTEMA: Través cruzado. Afastando no eixo paralelo ({lado})."
                elif v_phase == 4:
                    explanation = f"SISTEMA: Distância ideal atingida. Inicie curva BASE à {lado} para a proa {int(base_hdg):03}°."
                elif v_phase == 5:
                    if nav_state.get("base_turn_started"):
                        explanation = f"SISTEMA: Curva BASE à {lado} em andamento. Mantenha proa {int(base_hdg):03}° até interceptar o curso final {final_crs:03}°."
                    else:
                        target_dist_str = "10 NM" if gs_active else "1.0 NM"
                        explanation = f"SISTEMA: {target_dist_str} atingidos. Inicie curva à {lado} para a BASE na proa {int(base_hdg):03}°."
                elif v_phase == 6:
                    if is_ai_active and nav_state.get("ai_reason"):
                        explanation = f"IA: {nav_state['ai_reason']}"
                    else:
                        explanation = f"SISTEMA: Interceptando inbound na proa {int(target_hdg):03}°. Siga a proa de interceptação sugerida."
            else:
                if v_phase == 7:
                    if is_ai_active and nav_state.get("ai_reason"):
                        explanation = f"IA: {nav_state['ai_reason']}"
                    else:
                        fd_is_visible = nav_state.get("active", False) and not nav_state.get("missed_approach_climbing", False)
                        if fd_is_visible:
                            explanation = "IA: Inbound estabelecido. Siga o Flight Director e mantenha o eixo de aproximação."
                        else:
                            explanation = "IA: Inbound estabelecido. Mantenha o eixo de aproximação e acompanhe a descida."
                elif abs(xtk) > self.xtk_tolerance:
                    lado_txt = "DIREITA" if xtk > 0 else "ESQUERDA"
                    current_hdg = data.get("heading", 0)
                    diff_hdg = (target_hdg - current_hdg + 180) % 360 - 180
                    correcao_txt = "DIREITA" if diff_hdg > 0 else "ESQUERDA"
                    if is_ai_active and nav_state.get("ai_reason"):
                        explanation = f"IA: {nav_state['ai_reason']}"
                    elif nav_mode == "route" and waypoints and 0 <= active_wp_idx < len(waypoints):
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
                    if nav_mode == "route" and waypoints and 0 <= active_wp_idx < len(waypoints):
                        wp_name = waypoints[active_wp_idx].get("name", f"WP{active_wp_idx+1}")
                        explanation = f"IA: Rota estável. Aeronave na perna em direção a {wp_name}. Mantendo parâmetros nominais."
                    else:
                        explanation = f"IA: Rota nominal estável dentro do limite de {self.xtk_tolerance} NM. Aeronave mantendo parâmetros recomendados."
            
            # ── MODO LNAV: mensagem de correção de curso em tempo real na final/rota normal ──
            if log_on and (v_phase == 0 or v_phase >= 6):
                if is_ai_active and nav_state.get("ai_reason"):
                    explanation = f"IA: {nav_state['ai_reason']}"
                else:
                    rwy_crs = int(round(course)) % 360
                    cur_crs = int(round(nav_state.get("log_current_course", 0.0))) % 360
                    tgt_crs = nav_state.get("log_target_course", rwy_crs)
                    dev_val = nav_state.get("log_deviation", 0.0)
                    lado_dev = "DIREITA" if dev_val > 0 else "ESQUERDA"
                    lado_xtk = "DIREITA" if xtk > 0 else "ESQUERDA"
                    lado_curva = "ESQUERDA" if nav_state.get("log_turn") == "L" else "DIREITA"
                    intercepting = nav_state.get("log_intercepting", False)
                    prefix = "IA" if is_ai_active else "BACKUP"
                    if nav_state.get("log_correcting") and intercepting:
                        explanation = (f"{prefix}: Desvio de {abs(xtk):.2f} NM à {lado_xtk}. Curve à {lado_curva} para o curso {tgt_crs:03}° "
                                       f"e intercepte o eixo.")
                    elif nav_state.get("log_correcting"):
                        explanation = (f"{prefix}: Eixo {rwy_crs:03}°, proa atual {cur_crs:03}° "
                                       f"({abs(dev_val):.0f}° à {lado_dev}). Faça uma curva para a {lado_curva} "
                                       f"para o curso {tgt_crs:03}°.")
                    elif intercepting:
                        explanation = (f"{prefix}: Desvio de {abs(xtk):.2f} NM à {lado_xtk}. "
                                       f"Mantenha o curso {tgt_crs:03}° para interceptar suavemente.")
                    else:
                        explanation = (f"{prefix}: Sobre o eixo ({abs(xtk):.2f} NM), curso {cur_crs:03}° "
                                       f"(eixo {rwy_crs:03}°). Mantenha o curso.")
            
            # ── Distância e Razão de Descida até a cabeceira / Waypoint ──
            # Ocultada durante pernas de circuito (1 a 5) para manter apenas a distância única no topo
            if not (1 <= v_phase <= 5):
                if gs_active and gs_dist is not None:
                    dist_color = GREEN if gs_dist <= 10.0 else YELLOW
                    dist_txt = f"DIST CABECEIRA: {gs_dist:.1f} NM"
                    self.draw_text(surface, dist_txt, (ai_card_rect.centerx, ai_card_y + 300), self.f_btn, dist_color, align="center")
                elif nav_mode == "route" and dist_to_wp is not None:
                    wp_target_name = waypoints[active_wp_idx].get("name", "ALVO") if (waypoints and 0 <= active_wp_idx < len(waypoints)) else "ALVO"
                    dist_txt = f"DISTÂNCIA ATÉ {wp_target_name}: {dist_to_wp:.1f} NM"
                    self.draw_text(surface, dist_txt, (ai_card_rect.centerx, ai_card_y + 300), self.f_btn, GREEN, align="center")

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
            
            exp_start_y = ai_card_y + 200 if getattr(self, 'btn_side_left', None) is not None else ai_card_y + 155
            for index, line in enumerate(lines):
                self.draw_text(surface, line, (620, exp_start_y + (index * 26)), self.f_msg, WHITE)

            # ── Botão Manual de Próximo Waypoint (apenas Waypoints Manuais e com Flight Director ativo) ──
            if (not is_ma_climbing and nav_mode == "route" and not gs_active
                    and not nav_state.get("active_procedure")
                    and v_phase in (0, 5, 6, 7)
                    and waypoints and 0 <= active_wp_idx < len(waypoints) - 1):
                next_wp_obj = waypoints[active_wp_idx + 1]
                next_wp_name = next_wp_obj.get("name", f"WYP{active_wp_idx + 2:02d}")
                self.btn_next_wp = pygame.Rect(710, 748, 460, 56)
                is_hover_next = self.btn_next_wp.collidepoint(mouse_pos)
                btn_next_bg = (30, 85, 165) if is_hover_next else (20, 60, 125)
                pygame.draw.rect(surface, btn_next_bg, self.btn_next_wp, border_radius=12)
                pygame.draw.rect(surface, YELLOW, self.btn_next_wp, 2, border_radius=12)
                self.draw_text(
                    surface,
                    f"PRÓXIMO WAYPOINT ({next_wp_name})",
                    (self.btn_next_wp.centerx - 16, self.btn_next_wp.centery),
                    self.f_btn,
                    WHITE,
                    align="center"
                )
                # Seta gráfica para a direita (compatível com qualquer fonte)
                ax = self.btn_next_wp.right - 36
                ay = self.btn_next_wp.centery
                pygame.draw.line(surface, YELLOW, (ax - 14, ay), (ax + 4, ay), 4)
                pygame.draw.polygon(surface, YELLOW, [
                    (ax + 12, ay),
                    (ax + 1, ay - 9),
                    (ax + 1, ay + 9)
                ])

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
        self.draw_text(surface, "NAVEGAÇÃO VERTICAL", (panel_rect.centerx, 48), self.f_title, YELLOW, align="center")

        # Altitude Atual (em destaque com espaçamento ampliado)
        current_alt = nav_state.get("_smooth_alt", data.get('alt', 0))
        self.draw_text(surface, "ALTITUDE ATUAL", (panel_rect.centerx, 82), self.f_label, WHITE, align="center")
        self.draw_text(surface, f"{int(current_alt)} FT", (panel_rect.centerx, 122), self.f_data, WHITE, align="center")

        # ── Calibrar Altitude (Offset) ─────────────────────────────
        is_cal_active = (active_input == "vnav_cal")
        cal_border = BLUE if is_cal_active else GRAY
        pygame.draw.rect(surface, BOX_BG, self.vnav_cal_input_rect, border_radius=10)
        pygame.draw.rect(surface, cal_border, self.vnav_cal_input_rect, 2, border_radius=10)
        self.draw_text(surface, "Calibrar ALT (FT):", (self.vnav_cal_input_rect.x, self.vnav_cal_input_rect.y - 22), self.f_label, WHITE)

        display_cal = vnav_cal_text + ("|" if is_cal_active and (pygame.time.get_ticks() // 400) % 2 == 0 else "")
        if not vnav_cal_text and not is_cal_active:
            self.draw_text(surface, "ex: 2100", (self.vnav_cal_input_rect.x + 12, self.vnav_cal_input_rect.y + 9), self.f_small, GRAY)
        else:
            self.draw_text(surface, display_cal, (self.vnav_cal_input_rect.x + 12, self.vnav_cal_input_rect.y + 9), self.f_btn, WHITE)
        
        # Botão Ativar/Desativar Offset
        is_calibrated = nav_state.get("alt_calibrated", False)
        toggle_base = BTN_ON_GREEN if is_calibrated else (80, 80, 80)
        toggle_color = (min(255, toggle_base[0]+25), min(255, toggle_base[1]+25), min(255, toggle_base[2]+25)) \
                       if self.btn_toggle_vnav_cal.collidepoint(mouse_pos) else toggle_base
        pygame.draw.rect(surface, toggle_color, self.btn_toggle_vnav_cal, border_radius=10)
        pygame.draw.rect(surface, WHITE, self.btn_toggle_vnav_cal, 1, border_radius=10)
        toggle_txt = "ON" if is_calibrated else "OFF"
        self.draw_text(surface, toggle_txt, self.btn_toggle_vnav_cal.center, self.f_btn, WHITE, align="center")

        # ── Altitude Desejada ──────────────────────────────────────
        is_alt_active = (active_input == "vnav_alt")
        alt_border = BLUE if is_alt_active else GRAY
        pygame.draw.rect(surface, BOX_BG, self.vnav_alt_input_rect, border_radius=10)
        pygame.draw.rect(surface, alt_border, self.vnav_alt_input_rect, 2, border_radius=10)
        self.draw_text(surface, "Altitude Desejada (FT):", (self.vnav_alt_input_rect.x, self.vnav_alt_input_rect.y - 22), self.f_label, WHITE)

        display_alt = vnav_alt_text + ("|" if is_alt_active and (pygame.time.get_ticks() // 400) % 2 == 0 else "")
        if not vnav_alt_text and not is_alt_active:
            val = nav_state.get("vnav_target_alt")
            txt = f"{int(val)}" if val is not None else "ex: 5000"
            color = WHITE if val is not None else GRAY
            self.draw_text(surface, txt, (self.vnav_alt_input_rect.x + 12, self.vnav_alt_input_rect.y + 9), self.f_btn if val is not None else self.f_label, color)
        else:
            self.draw_text(surface, display_alt, (self.vnav_alt_input_rect.x + 12, self.vnav_alt_input_rect.y + 9), self.f_btn, WHITE)

        is_alt_custom = nav_state.get("vnav_custom_set", False)
        btn_alt_base = BTN_ON_GREEN if is_alt_custom else (80, 80, 80)
        btn_alt_color = (min(255, btn_alt_base[0]+25), min(255, btn_alt_base[1]+25), min(255, btn_alt_base[2]+25)) \
                        if self.btn_set_vnav_alt.collidepoint(mouse_pos) else btn_alt_base
        pygame.draw.rect(surface, btn_alt_color, self.btn_set_vnav_alt, border_radius=10)
        pygame.draw.rect(surface, WHITE, self.btn_set_vnav_alt, 1, border_radius=10)
        btn_alt_label = "ON" if is_alt_custom else "OFF"
        self.draw_text(surface, btn_alt_label, self.btn_set_vnav_alt.center, self.f_btn, WHITE, align="center")

        # ── Variação Aceita ────────────────────────────────────────
        is_tol_active = (active_input == "vnav_tol")
        tol_border = BLUE if is_tol_active else GRAY
        pygame.draw.rect(surface, BOX_BG, self.vnav_tol_input_rect, border_radius=10)
        pygame.draw.rect(surface, tol_border, self.vnav_tol_input_rect, 2, border_radius=10)
        self.draw_text(surface, "Variação Aceita (+/- FT):", (self.vnav_tol_input_rect.x, self.vnav_tol_input_rect.y - 22), self.f_label, WHITE)

        display_tol = vnav_tol_text + ("|" if is_tol_active and (pygame.time.get_ticks() // 400) % 2 == 0 else "")
        if not vnav_tol_text and not is_tol_active:
            # Exibe 200 como valor padrão do sistema em branco
            self.draw_text(surface, "200", (self.vnav_tol_input_rect.x + 12, self.vnav_tol_input_rect.y + 9), self.f_btn, WHITE)
        else:
            self.draw_text(surface, display_tol, (self.vnav_tol_input_rect.x + 12, self.vnav_tol_input_rect.y + 9), self.f_btn, WHITE)

        is_tol_on = nav_state.get("vnav_tol_custom", False)
        btn_tol_base = BTN_ON_GREEN if is_tol_on else (80, 80, 80)
        btn_tol_color = (min(255, btn_tol_base[0]+25), min(255, btn_tol_base[1]+25), min(255, btn_tol_base[2]+25)) \
                        if self.btn_set_vnav_tol.collidepoint(mouse_pos) else btn_tol_base
        pygame.draw.rect(surface, btn_tol_color, self.btn_set_vnav_tol, border_radius=10)
        pygame.draw.rect(surface, WHITE, self.btn_set_vnav_tol, 1, border_radius=10)
        btn_tol_txt = "ON" if is_tol_on else "OFF"
        self.draw_text(surface, btn_tol_txt, self.btn_set_vnav_tol.center, self.f_btn, WHITE, align="center")

        # ── STATUS DO VNAV ─────────────────────────────────────────
        vnav_gs = nav_state.get("vnav_gs_alt")
        target_alt = vnav_gs if vnav_gs is not None else nav_state.get("vnav_target_alt")
        tolerance = max(10.0, float(nav_state.get("vnav_tolerance", 200) or 200))
        status_box = pygame.Rect(1380, 440, 360, 145)
        v_phase_vnav = nav_state.get("vectoring_phase", 0)
        is_ma_climbing = nav_state.get("missed_approach_climbing", False)
        is_nav_active = nav_state.get("active", False)
        is_custom_alt = nav_state.get("vnav_custom_set", False)

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
            self.draw_text(surface, "ARREMETIDA - SUBINDO", (status_box.centerx, status_box.y + 30), self.f_title, (255, 165, 0), align="center")
            self.draw_text(surface, f"META: {int(ma_target)} FT", (status_box.centerx, status_box.y + 70), self.f_btn, WHITE, align="center")
            self.draw_text(surface, f"FALTAM: {int(ft_remaining)} FT", (status_box.centerx, status_box.y + 110), self.f_btn, (255, 200, 100), align="center")

        # Exibe status VNAV normal apenas se:
        # 1) Houver altitude manual travada (ON), OU
        # 2) A navegação estiver ATIVA (active=True) e fora das fases de vetoramento inicial (1 a 5)
        elif target_alt is not None and (is_custom_alt or (is_nav_active and not (1 <= v_phase_vnav <= 5))):
            diff = current_alt - target_alt
            abs_diff = abs(diff)

            if abs_diff > tolerance:
                # Alerta Vermelho com comando claro SUBA/DESÇA
                # Padrão Internacional: pisca por 10 segundos e depois fica estático vermelho
                direcao = "ABAIXO" if diff < 0 else "ACIMA"
                comando = "SUBA" if diff < 0 else "DESÇA"

                now_t = time.time()
                alt_start = nav_state.get("_alt_alert_start")
                last_cmd = nav_state.get("_alt_alert_cmd")

                if alt_start is None or last_cmd != comando:
                    nav_state["_alt_alert_start"] = now_t
                    nav_state["_alt_alert_cmd"] = comando
                    alt_start = now_t

                elapsed = now_t - alt_start
                if elapsed < 10.0:
                    blink_on = ((pygame.time.get_ticks() // 350) % 2 == 0)
                else:
                    blink_on = True

                if blink_on:
                    pygame.draw.rect(surface, (70, 15, 15), status_box, border_radius=15)
                    pygame.draw.rect(surface, RED, status_box, 3, border_radius=15)
                    self.draw_text(surface, "ALERTA DE ALTITUDE", (status_box.centerx, status_box.y + 24), self.f_title, RED, align="center")
                    self.draw_text(surface, f"{int(abs_diff)} FT {direcao} DO ALVO", (status_box.centerx, status_box.y + 64), self.f_btn, WHITE, align="center")
                    self.draw_text(surface, comando, (status_box.centerx, status_box.y + 102), self.f_title, YELLOW, align="center")
                else:
                    # Fase apagada do piscar rítmico durante os 10s iniciais
                    pygame.draw.rect(surface, BOX_BG, status_box, border_radius=15)
                    pygame.draw.rect(surface, (70, 15, 15), status_box, 1, border_radius=15)
            else:
                # Status OK (Verde) - reseta o timer do alerta para piscar se sair novamente
                nav_state["_alt_alert_start"] = None
                nav_state["_alt_alert_cmd"] = None
                pygame.draw.rect(surface, (20, 45, 25), status_box, border_radius=15)
                pygame.draw.rect(surface, GREEN, status_box, 2, border_radius=15)
                self.draw_text(surface, "ALTITUDE MANTIDA", (status_box.centerx, status_box.y + 35), self.f_title, GREEN, align="center")
                self.draw_text(surface, f"ALVO: {int(target_alt)} FT", (status_box.centerx, status_box.y + 85), self.f_btn, WHITE, align="center")
        else:
            # Standby - reseta o timer do alerta
            nav_state["_alt_alert_start"] = None
            nav_state["_alt_alert_cmd"] = None
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

    def draw_touch_keypad(self, surface, mouse_pos):
        """Desenha o teclado touch sobreposto se estiver ativo."""
        if self.touch_keypad and self.touch_keypad.active:
            self.touch_keypad.draw(surface, mouse_pos)

    def draw_attitude_panel(self, surface, mouse_pos, pitch, bank, calib):
        """Desenha o painel de calibração/nivelamento de atitude se ativo."""
        if hasattr(self, 'attitude_panel') and self.attitude_panel and self.attitude_panel.active:
            if not calib:
                cal_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sensor_calibration.json")
                if os.path.exists(cal_path):
                    try:
                        with open(cal_path, "r", encoding="utf-8") as f:
                            calib = json.load(f)
                    except Exception:
                        calib = {}
            self.attitude_panel.draw(surface, mouse_pos, pitch, bank, calib)

# ============================================================================
# 7. CONTROLADOR DE NAVEGAÇÃO E LOOP PRINCIPAL (monitor_lateral.py)
# ============================================================================
# Módulos locais consolidados e disponíveis no mesmo namespace global
def get_next_log_filename(log_dir):
    """Determina o proximo nome sequencial de arquivo de log de voo (.csv) garantindo numeracao progressiva e sem sobrescritas."""
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    files = glob.glob(os.path.join(log_dir, "log_*.csv"))
    
    max_major = 1
    max_minor = -1
    
    for f in files:
        basename = os.path.basename(f)
        name = basename.replace("log_", "").replace(".csv", "")
        try:
            parts = name.split('.')
            major = int(parts[0])
            minor = int(parts[1])
            if major > max_major:
                max_major = major
                max_minor = minor
            elif major == max_major and minor > max_minor:
                max_minor = minor
        except:
            pass
            
    if max_minor == -1:
        return os.path.join(log_dir, "log_1.00.csv")
        
    next_minor = max_minor + 1
    next_major = max_major
    if next_minor > 99:
        next_major += 1
        next_minor = 0
        
    return os.path.join(log_dir, f"log_{next_major}.{next_minor:02d}.csv")

# Caixas de Dados de Telemetria (HDG, TRK, SPD, ALT)
HDG_RECT = pygame.Rect(198, 22, 165, 82)
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
    """
    Funcao principal de execucao do NAVEG-AI. Inicializa os sistemas, gerencia o loop de eventos a 30 FPS,
    processa telemetria, computa trajetoria de navegacao lateral e vertical (VNAV), aciona o Flight Director e grava logs de voo.
    """
    # Reseta o offset temporário de proa da pista para 0° ao iniciar (inicia sempre na proa natural)
    try:
        cal_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sensor_calibration.json")
        if os.path.exists(cal_path):
            with open(cal_path, "r", encoding="utf-8") as f:
                _c_init = json.load(f)
            if _c_init.get("heading_offset", 0.0) != 0.0:
                _c_init["heading_offset"] = 0.0
                with open(cal_path, "w", encoding="utf-8") as f:
                    json.dump(_c_init, f, indent=4)
                print("[INFO] Proa inicializada com offset zerado (proa natural do sensor).")
    except Exception:
        pass

    # Parsing de argumentos para selecionar a fonte de dados
    # Inicializa o parser de argumentos de linha de comando para configuracao do modo de voo
    parser = argparse.ArgumentParser(description="Monitor de Navegação Lateral - Voo Real")
    # Parametro --mode define a origem dos dados de telemetria ('pi', 'sim', 'mock', 'udp', 'auto')
    parser.add_argument("--mode", type=str, choices=["auto", "pi", "sim", "mock", "udp"], default="pi",
                        help="Seleciona a fonte de telemetria: 'pi' (Sensores Raspberry Pi), 'sim' (MSFS), 'udp' (Receptor WiFi/Rede), 'mock' (Simulado)")
    # Realiza o parsing dos argumentos passados via terminal ou atalho .bat
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

    # Instancia o provedor SimConnect para comunicacao com o Microsoft Flight Simulator
    if mode_chosen == "sim":
        source = SimConnectSource()
    # Instancia o provedor de hardware fisico para Raspberry Pi (BNO055 e GPS Neo-M8N)
    elif mode_chosen == "pi":
        # Porta padrão para o GPS Neo M8N na Pi
        source = PiSensorSource(gps_port="/dev/ttyAMA0", gps_baud=9600)
    # Instancia o receptor de pacotes UDP na porta 5005
    elif mode_chosen == "udp":
        source = UDPSource(port=5005)
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
    # Inicializa todos os subsistemas da biblioteca Pygame (graficos, fontes, timers)
    pygame.init()
    # Cria a janela principal do Pygame com suporte a redimensionamento dinamico
    screen = pygame.display.set_mode(WINDOW_SIZE, pygame.RESIZABLE)
    # Cria a superficie de desenho interna com resolucao de referencia nativa (1800x900)
    canvas = pygame.Surface(BASE_WINDOW_SIZE)
    window_size = WINDOW_SIZE
    pygame.display.set_caption(f"Naveg-AI Monitor - Telemetria [TESTADO] ({mode_chosen.upper()})")

    # Fontes básicas para as caixas do PFD
    box_title_font = pygame.font.SysFont("Segoe UI", 20, bold=True)
    box_value_font = pygame.font.SysFont("Consolas", 36, bold=True)
    
    # Relogio de controle de taxa de quadros (30 FPS estaveis)
    clock = pygame.time.Clock()
    # Instancia a interface grafica do Primary Flight Display (PFD / HUD)
    gui = PFDGui(base_size=BASE_WINDOW_SIZE)

    # Estado de Navegação Lateral
    # Todos os blocos do código foram totalmente comentados.
    nav_state = {
        "system_on": False,         # Sistema ligado (processando IA e botões), mas não necessariamente rastreando.
        "active": False,            # Monitoramento lateral ativo (curso travado).
        "lat_orig": 0.0,                    # Latitude de referencia do fixo de origem do segmento
        "lon_orig": 0.0,                    # Longitude de referencia do fixo de origem do segmento
        "course": 0.0,              # Curso magnético exibido na interface (ex: 172°)
        "course_true": 0.0,         # Curso verdadeiro usado nos cálculos trigonométricos
        "xtk": 0.0,                 # Cross-track error calculado em milhas náuticas (NM)
        "target_heading": 0,        # Proa magnética sugerida pela IA de correção
        "current_heading": 0.0,     # Proa magnética atual da aeronave a 30 FPS
        "fd_steering_mode": True,   # True: Flight Director guiado por Comando de Curva da IA (Opção 2)
        "xtk_tolerance": 0.3,       # Tolerância de desvio lateral em milhas náuticas (padrão: 0.3 NM)
        # ── Campos de Waypoints ──────────────────────────────────────────────
        "recorded_waypoints": [],   # Lista de dicts: {name, lat, lon, course, course_true}
        "navigation_mode": "manual",# 'manual' ou 'route'
        "active_waypoint_index": 0, # Índice do waypoint alvo atual no modo 'route'
        # ── Campos de Vetoração (Procedural Turn) ─────────────────────────────
        "vectoring_phase": 0,           # 0: Inativo, 1: Aguardando curva, 2: Perna de afastamento, 3: Segunda curva
        "vectoring_initial_heading": 0.0,   # Proa inicial da aeronave ao iniciar vetoracao RNAV
        "vectoring_turn_dir": 1,        # 1: Direita, -1: Esquerda
        "vectoring_target_hdg": 0.0,        # Proa calculada para interceptacao da perna base
        "vnav_target_alt": None,            # Altitude alvo calculada pelo perfil de descida VNAV
        "vnav_tolerance": 200,              # Tolerancia de desvio vertical em pes (+/- 200 ft)
        "prev_dist_to_wp": None,            # Distancia do frame anterior para detectar fly-by
        "alt_offset": 0.0,                  # Offset barometrico aplicado para zerar a elevacao
        "alt_calibrated": False,            # Flag que confirma a calibracao altimetrica
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
        "saved_gs_approach": None,          # Backup da rampa para recarregar depois
        "circuit_side_locked": False,       # True quando o lado do circuito veio da rampa salva (guia p/ paralela do lado certo)
        "pattern_course": None,             # Curso (mag) de referência do circuito = curso do waypoint alvo (usado na tela)
        "is_return_approach": False,        # True apenas quando carrega rampa salva após waypoints manuais (mostra botões/4 NM)
        "gs_takeoff_active": False,         # True na decolagem do chão (INICIAR 50FT / SALVAR RAMPA no solo)
        "is_go_around": False,              # True durante subida de arremetida (mantém proa de pista até a 1ª curva)
        "downwind_established": False,     # True após o avião voar alinhado na perna do vento (evita disparar base na decolagem)
        # ── Modo LOG (monitoramento de curso em tempo real na final) ───────────
        "log_mode": False,                  # Botão LOG ligado/desligado (desligado = comportamento padrão 0.3 NM)
        "log_final_active": False,          # True quando LOG ligado e aeronave estabelecida na final
        "log_correcting": False,            # True quando |desvio de curso| >= 2° (histerese até <= 1°)
        
        # ── LOG CSV ──
        "csv_logging_active": False,        # Flag que comanda a gravacao ativa no arquivo CSV
        "csv_log_filename": None,           # Nome e caminho do arquivo de log oficial de voo
        "csv_last_time": 0.0,               # Timestamp da gravacao da ultima linha de telemetria
    }

    # Estado da entrada de dados na interface (três caixas de texto com foco dinâmico)
    active_input = "none"       # Foco ativo: "course", "tolerance", "timer", "vnav_alt", "vnav_tol", "vnav_cal", "gs_rwy_alt" ou "none"
    course_text = ""            # Buffer para digitação do curso
    tolerance_text = "0.3"      # Buffer para digitação da tolerância (iniciado com o padrão 0.3)
    vnav_alt_text = ""          # Buffer para digitação da altitude VNAV
    vnav_tol_text = "200"       # Buffer para digitação da tolerância VNAV (padrão 200 FT)
    vnav_cal_text = ""          # Buffer para digitação do Offset de Altitude
    gs_rwy_alt_text = ""        # Buffer para digitação da altitude da pista (Glide Slope Sintético)
    hdg_cal_text = ""           # Buffer para digitação e calibração de proa magnética
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
    
    # Inicializa o Co-Piloto Neural de Reinterceptação (Thread assíncrona local)
    ai_copilot = NavegAICopilot(update_interval=2.0)

    running = True
    # ========================================================================
    # LOOP PRINCIPAL DE VOO E RENDERIZACAO DE AVIONICA (30 FRAMES POR SEGUNDO)
    # ========================================================================
    while running:
        # Aquisição de dados de telemetria
        # Coleta a leitura mais recente de telemetria da aeronave (a cada iteracao do loop)
        data = source.get_data()
        if "heading" in data:
            nav_state["current_heading"] = data["heading"]
            nav_state["current_bank"] = data.get("bank", 0.0)
            
            # Taxa de Curva (Turn Rate) via Proa/GPS (100% imune à posição física da Raspberry no colo)
            now_t = time.time()
            if "_last_hdg_t" in nav_state:
                dt_hdg = now_t - nav_state["_last_hdg_t"]
                if dt_hdg >= 0.08:
                    d_hdg = (data["heading"] - nav_state["_last_hdg_val"] + 180.0) % 360.0 - 180.0
                    # Limita a taxa física entre -6°/s e +6°/s (evita picos causados por tremores ou ruídos)
                    raw_turn_rate = max(-6.0, min(6.0, d_hdg / dt_hdg))
                    old_rate = nav_state.get("turn_rate_dps", 0.0)
                    # Filtro passa-baixa 85/15: leitura de curva super estável e sem espasmos
                    nav_state["turn_rate_dps"] = old_rate * 0.85 + raw_turn_rate * 0.15
                    nav_state["_last_hdg_val"] = data["heading"]
                    nav_state["_last_hdg_t"] = now_t
            else:
                nav_state["_last_hdg_val"] = data["heading"]
                nav_state["_last_hdg_t"] = now_t
                nav_state["turn_rate_dps"] = 0.0
        
        # Aplica o offset de calibração se estiver ativo
        if nav_state.get("alt_calibrated"):
            data["alt"] += nav_state.get("alt_offset", 0.0)

        # Suavização adaptativa (low-pass filter) para horizonte artificial e instrumentos ultra fluidos
        if "_smooth_pitch" not in nav_state:
            nav_state["_smooth_pitch"] = float(data.get("pitch", 0.0))
            nav_state["_smooth_bank"] = float(data.get("bank", 0.0))
            nav_state["_smooth_speed"] = float(data.get("speed", 0.0))
            nav_state["_smooth_alt"] = float(data.get("alt", 0.0))
            nav_state["_smooth_hdg"] = float(data.get("heading", 0.0))
            nav_state["_smooth_trk"] = float(data.get("track", data.get("heading", 0.0)))

        # Fatores de suavização a 30 FPS:
        # Horizonte (Pitch/Bank): 0.65 -> resposta rápida (~95% em 2 frames) com filtragem de trepidação mecânica
        # Velocidade e Altitude: transição suave e contínua entre os ticks do GPS
        nav_state["_smooth_pitch"] += (float(data.get("pitch", 0.0)) - nav_state["_smooth_pitch"]) * 0.65
        nav_state["_smooth_bank"] += (float(data.get("bank", 0.0)) - nav_state["_smooth_bank"]) * 0.65
        raw_spd_val = float(data.get("speed", 0.0))
        if raw_spd_val < 2.8:
            nav_state["_smooth_speed"] = 0.0
        else:
            nav_state["_smooth_speed"] += (raw_spd_val - nav_state["_smooth_speed"]) * 0.35

        raw_alt_val = float(data.get("alt", 0.0))
        if abs(raw_alt_val - nav_state["_smooth_alt"]) > 150.0:
            nav_state["_smooth_alt"] = raw_alt_val
        else:
            nav_state["_smooth_alt"] += (raw_alt_val - nav_state["_smooth_alt"]) * 0.15

        # Suavização angular contínua da proa e do rumo (lidando com 0° <-> 360°)
        raw_hdg = float(data.get("heading", 0.0))
        cur_shdg = nav_state["_smooth_hdg"]
        diff_hdg = (raw_hdg - cur_shdg + 180.0) % 360.0 - 180.0
        nav_state["_smooth_hdg"] = (cur_shdg + diff_hdg * 0.45) % 360.0

        if raw_spd_val < 3.5 and not data.get("is_real_pi", False):
            # (Fontes nao-Pi: simulador/UDP/mock.) A Pi ja entrega o TRK continuo em get_data().
            data["track"] = raw_hdg
            nav_state["_smooth_trk"] = nav_state["_smooth_hdg"]
        else:
            raw_trk = float(data.get("track", raw_hdg))
            cur_strk = nav_state.get("_smooth_trk", raw_trk)
            diff_trk = (raw_trk - cur_strk + 180.0) % 360.0 - 180.0
            nav_state["_smooth_trk"] = (cur_strk + diff_trk * 0.35) % 360.0

        # Gerenciamento de eventos
        # Processamento da fila de eventos do Pygame (mouse, teclado, redimensionamento)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.VIDEORESIZE:
                window_size = event.size
    # Cria a janela principal do Pygame com suporte a redimensionamento dinamico
                screen = pygame.display.set_mode(window_size, pygame.RESIZABLE)

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    if hasattr(gui, 'touch_keypad') and gui.touch_keypad:
                        gui.touch_keypad.handle_mouse_up()
                    if hasattr(gui, 'attitude_panel') and gui.attitude_panel:
                        gui.attitude_panel.handle_mouse_up()

            elif event.type == pygame.MOUSEMOTION:
                mouse_pos = window_to_canvas_pos(event.pos, window_size)
                if hasattr(gui, 'touch_keypad') and gui.touch_keypad and gui.touch_keypad.dragging:
                    gui.touch_keypad.handle_motion(mouse_pos)
                if hasattr(gui, 'attitude_panel') and gui.attitude_panel and gui.attitude_panel.dragging:
                    gui.attitude_panel.handle_motion(mouse_pos)

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = window_to_canvas_pos(event.pos, window_size)
                if event.button == 1:
                    # 0. Se o painel de atitude estiver ativo, processa o clique nele primeiro
                    if hasattr(gui, 'attitude_panel') and gui.attitude_panel and gui.attitude_panel.active:
                        try:
                            att_res = gui.attitude_panel.handle_click(mouse_pos)
                            if att_res["action"] != "none":
                                if att_res["action"] in ("drag_start", "consumed", "close"):
                                    continue
                                if att_res["action"] == "zero_tare":
                                    raw_p = getattr(source, 'last_true_pitch', data.get("pitch", 0.0))
                                    raw_b = getattr(source, 'last_true_bank', data.get("bank", 0.0))
                                    cal_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sensor_calibration.json")
                                    calib = {}
                                    if os.path.exists(cal_path):
                                        try:
                                            with open(cal_path, "r", encoding="utf-8") as f:
                                                calib = json.load(f)
                                        except Exception:
                                            calib = {}
                                    calib["pitch_offset"] = round(raw_p, 4)
                                    calib["bank_offset"] = round(raw_b, 4)
                                    calib["timestamp"] = time.time()
                                    calib["calibrated_via"] = "Touch Zero Tare (Repouso)"
                                    with open(cal_path, "w", encoding="utf-8") as f:
                                        json.dump(calib, f, indent=4)
                                    if hasattr(source, "calibration"):
                                        source.calibration = dict(calib)
                                    gui.attitude_panel.feedback_msg = "Nivelado com Sucesso! (0.0 / 0.0)"
                                    gui.attitude_panel.feedback_time = time.time()
                                    try:
                                        print(f"[OK] Atitude Nivelada: Pitch_offset={raw_p:+.2f}, Bank_offset={raw_b:+.2f}")
                                    except Exception:
                                        pass
                                elif att_res["action"] == "toggle_pitch":
                                    cal_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sensor_calibration.json")
                                    calib = {}
                                    if os.path.exists(cal_path):
                                        try:
                                            with open(cal_path, "r", encoding="utf-8") as f:
                                                calib = json.load(f)
                                        except Exception:
                                            calib = {}
                                    new_val = not calib.get("invert_pitch", False)
                                    calib["invert_pitch"] = new_val
                                    with open(cal_path, "w", encoding="utf-8") as f:
                                        json.dump(calib, f, indent=4)
                                    if hasattr(source, "calibration"):
                                        source.calibration = dict(calib)
                                    gui.attitude_panel.feedback_msg = f"Inverter Pitch: {'SIM' if new_val else 'NAO'}"
                                    gui.attitude_panel.feedback_time = time.time()
                                    try:
                                        print(f"[OK] Inverter Pitch alterado para: {new_val}")
                                    except Exception:
                                        pass
                                elif att_res["action"] == "toggle_bank":
                                    cal_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sensor_calibration.json")
                                    calib = {}
                                    if os.path.exists(cal_path):
                                        try:
                                            with open(cal_path, "r", encoding="utf-8") as f:
                                                calib = json.load(f)
                                        except Exception:
                                            calib = {}
                                    new_val = not calib.get("invert_bank", False)
                                    calib["invert_bank"] = new_val
                                    with open(cal_path, "w", encoding="utf-8") as f:
                                        json.dump(calib, f, indent=4)
                                    if hasattr(source, "calibration"):
                                        source.calibration = dict(calib)
                                    gui.attitude_panel.feedback_msg = f"Inverter Bank: {'SIM' if new_val else 'NAO'}"
                                    gui.attitude_panel.feedback_time = time.time()
                                    try:
                                        print(f"[OK] Inverter Bank alterado para: {new_val}")
                                    except Exception:
                                        pass
                                elif att_res["action"] == "toggle_heading":
                                    cal_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sensor_calibration.json")
                                    calib = {}
                                    if os.path.exists(cal_path):
                                        try:
                                            with open(cal_path, "r", encoding="utf-8") as f:
                                                calib = json.load(f)
                                        except Exception:
                                            calib = {}
                                    new_val = not calib.get("invert_heading", False)
                                    calib["invert_heading"] = new_val
                                    with open(cal_path, "w", encoding="utf-8") as f:
                                        json.dump(calib, f, indent=4)
                                    if hasattr(source, "calibration"):
                                        source.calibration = dict(calib)
                                    gui.attitude_panel.feedback_msg = f"Inverter Proa (180°): {'SIM' if new_val else 'NAO'}"
                                    gui.attitude_panel.feedback_time = time.time()
                                    try:
                                        print(f"[OK] Inverter Proa (HDG) alterado para: {new_val}")
                                    except Exception:
                                        pass
                                continue
                        except Exception as e:
                            try:
                                print(f"[ERRO ATTITUDE]: {e}")
                            except Exception:
                                pass
                            continue

                    # 1. Se o teclado touch estiver ativo, processa o clique nele
                    if gui.touch_keypad and gui.touch_keypad.active:
                        kp_res = gui.touch_keypad.handle_click(mouse_pos)
                        if kp_res["action"] in ("update", "drag_start", "consumed"):
                            if kp_res["action"] == "update":
                                if active_input == "tolerance":
                                    tolerance_text = kp_res["value"]
                                elif active_input == "vnav_alt":
                                    vnav_alt_text = kp_res["value"]
                                elif active_input == "course":
                                    course_text = kp_res["value"]
                                elif active_input == "gs_rwy_alt":
                                    gs_rwy_alt_text = kp_res["value"]
                                elif active_input == "vnav_tol":
                                    vnav_tol_text = kp_res["value"]
                                elif active_input == "vnav_cal":
                                    vnav_cal_text = kp_res["value"]
                                elif active_input == "cal_hdg":
                                    hdg_cal_text = kp_res["value"]
                            continue
                        elif kp_res["action"] == "commit":
                            field = kp_res["field"]
                            val_str = kp_res["value"]
                            if field == "course":
                                if val_str.isdigit():
                                    course_val = int(val_str) % 360
                                    if data.get("connected"):
                                        if nav_state["lat_orig"] == 0.0 and nav_state["lon_orig"] == 0.0:
                                            nav_state["lat_orig"] = data["lat"]
                                            nav_state["lon_orig"] = data["lon"]
                                        nav_state["course"] = float(course_val)
                                        mag_var = data.get("mag_var", 0.0)
                                        nav_state["course_true"] = (float(course_val) + mag_var) % 360
                                        nav_state["active"] = True
                                        print(f"Curso Travado via Touch: {course_val:03}°")
                                    course_text = f"{int(course_val):03}"
                            elif field == "tolerance":
                                try:
                                    tolerance_val = float(val_str) if val_str else 0.3
                                    if tolerance_val <= 0: tolerance_val = 0.3
                                    nav_state["xtk_tolerance"] = tolerance_val
                                    nav_state["xtk_custom_set"] = True
                                    tolerance_text = f"{tolerance_val:.2f}".rstrip('0').rstrip('.')
                                    print(f"Tolerância XTK atualizada via Touch: {tolerance_val} NM")
                                except ValueError: pass
                            elif field == "vnav_alt":
                                try:
                                    if val_str:
                                        alt_val = float(val_str)
                                        nav_state["vnav_target_alt"] = alt_val
                                        nav_state["vnav_custom_set"] = True
                                        vnav_alt_text = str(int(alt_val))
                                except ValueError: pass
                            elif field == "gs_rwy_alt":
                                try:
                                    if val_str:
                                        rwy_val = float(val_str)
                                        nav_state["gs_rwy_elev"] = rwy_val
                                        gs_rwy_alt_text = str(int(rwy_val))
                                        print(f"Elevação RWY atualizada via Touch: {rwy_val} FT")
                                except ValueError: pass
                            elif field == "vnav_tol":
                                try:
                                    tol_val = float(val_str) if val_str else 200.0
                                    if tol_val <= 0: tol_val = 200.0
                                    nav_state["vnav_tolerance"] = tol_val
                                    nav_state["vnav_tol_custom"] = True
                                    vnav_tol_text = str(int(tol_val))
                                    print(f"Variação VNAV atualizada via Touch: +/- {tol_val} FT")
                                except ValueError: pass
                            elif field == "vnav_cal":
                                try:
                                    if val_str:
                                        desired_alt = float(val_str)
                                        raw_alt = source.get_data().get("alt", 0)
                                        offset = desired_alt - raw_alt
                                        nav_state["alt_offset"] = offset
                                        nav_state["alt_calibrated"] = True
                                        vnav_cal_text = str(int(desired_alt))
                                        print(f"Altitude Calibrada via Touch: {desired_alt} FT (Offset: {offset})")
                                except ValueError: pass
                            elif field == "cal_hdg":
                                try:
                                    cal_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sensor_calibration.json")
                                    calib = {}
                                    if os.path.exists(cal_path):
                                        try:
                                            with open(cal_path, "r", encoding="utf-8") as f:
                                                calib = json.load(f)
                                        except Exception:
                                            pass
                                    if val_str in ("reset", "ZERAR", "zerar"):
                                        calib["heading_offset"] = 0.0
                                        calib["timestamp"] = time.time()
                                        calib["calibrated_via"] = "Reset to Sensor Natural Heading (ZERAR)"
                                        with open(cal_path, "w", encoding="utf-8") as f:
                                            json.dump(calib, f, indent=4)
                                        if hasattr(source, "calibration"):
                                            source.calibration["heading_offset"] = 0.0
                                        cur_hdg = int(round(data.get("heading", 0.0) - calib.get("heading_offset", 0.0))) % 360
                                        hdg_cal_text = f"{cur_hdg:03}"
                                        print("[OK] Offset de proa resetado para 0.0° (Proa natural do sensor restaurada).")
                                    elif val_str:
                                        desired_hdg = float(val_str) % 360.0
                                        current_hdg = data.get("heading", 0.0)
                                        old_offset = calib.get("heading_offset", 0.0)
                                        base_hdg = (current_hdg - old_offset) % 360.0
                                        new_offset = (desired_hdg - base_hdg) % 360.0
                                        calib["heading_offset"] = new_offset
                                        calib["timestamp"] = time.time()
                                        calib["calibrated_via"] = f"Touch Align ({int(desired_hdg):03}°)"
                                        with open(cal_path, "w", encoding="utf-8") as f:
                                            json.dump(calib, f, indent=4)
                                        if hasattr(source, "calibration"):
                                            source.calibration["heading_offset"] = new_offset
                                        hdg_cal_text = f"{int(desired_hdg):03}"
                                        print(f"[OK] Proa Sincronizada via Touch: {int(desired_hdg):03} (Offset: {new_offset:+.1f})")
                                except Exception as e:
                                    print(f"Erro ao calibrar proa: {e}")
                            active_input = "none"
                            continue
                        elif kp_res["action"] == "close":
                            active_input = "none"
                            continue
                        elif kp_res["action"] == "outside":
                            active_input = "none"
                            # Não executa continue: permite checar se o toque clicou em outro campo de entrada ou botão!

                    # 2. Foca na caixa clicada e abre o Teclado Touch correspondente
                    if gui.tolerance_input_rect.collidepoint(mouse_pos):
                        active_input = "tolerance"
                        if not (gui.touch_keypad.active and gui.touch_keypad.field_name == "tolerance"):
                            gui.touch_keypad.open("tolerance", "TOLERANCIA XTK (NM)", tolerance_text or "0.3", "miles", pos=(760, 246))
                    elif gui.vnav_alt_input_rect.collidepoint(mouse_pos):
                        active_input = "vnav_alt"
                        if not (gui.touch_keypad.active and gui.touch_keypad.field_name == "vnav_alt"):
                            gui.touch_keypad.open("vnav_alt", "ALTITUDE ALVO (FT)", vnav_alt_text, "altitude", pos=(1320, 420))
                    elif gui.course_input_rect.collidepoint(mouse_pos):
                        active_input = "course"
                        if not (gui.touch_keypad.active and gui.touch_keypad.field_name == "course"):
                            gui.touch_keypad.open("course", "CURSO DESEJADO (0-359)", course_text, "course", pos=(600, 246))
                    elif gui.gs_rwy_alt_input_rect.collidepoint(mouse_pos):
                        active_input = "gs_rwy_alt"
                        if not (gui.touch_keypad.active and gui.touch_keypad.field_name == "gs_rwy_alt"):
                            gui.touch_keypad.open("gs_rwy_alt", "ALTITUDE DA PISTA (FT)", gs_rwy_alt_text, "altitude", pos=(860, 246))
                    elif gui.vnav_tol_input_rect.collidepoint(mouse_pos):
                        active_input = "vnav_tol"
                        if not (gui.touch_keypad.active and gui.touch_keypad.field_name == "vnav_tol"):
                            gui.touch_keypad.open("vnav_tol", "VARIACAO ACEITA (+/- FT)", vnav_tol_text or "200", "altitude", pos=(1320, 420))
                    elif gui.vnav_cal_input_rect.collidepoint(mouse_pos):
                        active_input = "vnav_cal"
                        if not (gui.touch_keypad.active and gui.touch_keypad.field_name == "vnav_cal"):
                            gui.touch_keypad.open("vnav_cal", "CALIBRAR ALTITUDE (FT)", vnav_cal_text, "altitude", pos=(1320, 420))
                    elif HDG_RECT.collidepoint(mouse_pos):
                        active_input = "cal_hdg"
                        if not (gui.touch_keypad.active and gui.touch_keypad.field_name == "cal_hdg"):
                            cur_hdg = int(round(data.get("heading", 0.0))) % 360
                            hdg_cal_text = f"{cur_hdg:03}"
                            gui.touch_keypad.open("cal_hdg", "CALIBRAR PROA MAG (0-359)", hdg_cal_text, "course", pos=(198, 110))
                    elif ((mouse_pos[0] - 280)**2 + (mouse_pos[1] - 380)**2) <= 185**2:
                        active_input = "none"
                        if gui.touch_keypad:
                            gui.touch_keypad.close()
                        if hasattr(gui, 'attitude_panel'):
                            gui.attitude_panel.open()
                    else:
                        active_input = "none"
                        if gui.touch_keypad:
                            gui.touch_keypad.close()
                
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

                # Clique no botão TOGGLE da Tolerância XTK customizada (ON/OFF)
                if gui.btn_set_tolerance.collidepoint(mouse_pos):
                    is_xtk_on = nav_state.get("xtk_custom_set", False)
                    if not is_xtk_on:
                        # Liga tolerância personalizada (ON)
                        try:
                            tolerance_val = float(tolerance_text) if tolerance_text else 0.3
                            if tolerance_val <= 0:
                                tolerance_val = 0.3
                            nav_state["xtk_tolerance"] = tolerance_val
                            nav_state["xtk_custom_set"] = True
                            tolerance_text = f"{tolerance_val:.2f}".rstrip('0').rstrip('.')
                            print(f"Tolerância XTK personalizada (ON): {tolerance_val} NM")
                        except ValueError:
                            print("Valor de tolerância inválido.")
                    else:
                        # Desliga tolerância personalizada (OFF -> volta sempre para 0.3 NM)
                        nav_state["xtk_tolerance"] = 0.3
                        nav_state["xtk_custom_set"] = False
                        tolerance_text = "0.3"
                        print("Tolerância XTK restaurada para o padrão (OFF): 0.3 NM")
                    active_input = "none"

                # Clique no botão TOGGLE VNAV ALT (ON/OFF)
                if gui.btn_set_vnav_alt.collidepoint(mouse_pos):
                    is_alt_on = nav_state.get("vnav_custom_set", False)
                    if not is_alt_on:
                        # Liga altitude manual única (ON)
                        try:
                            if vnav_alt_text:
                                alt_val = float(vnav_alt_text)
                            elif nav_state.get("vnav_target_alt") is not None:
                                alt_val = float(nav_state["vnav_target_alt"])
                                vnav_alt_text = str(int(alt_val))
                            else:
                                alt_val = float(data.get("alt", 0))
                                vnav_alt_text = str(int(alt_val))
                            nav_state["vnav_target_alt"] = alt_val
                            nav_state["vnav_custom_set"] = True
                            print(f"Altitude Alvo VNAV manual travada (ON): {alt_val} FT")
                        except ValueError:
                            print("Altitude VNAV inválida.")
                    else:
                        # Desliga altitude manual (OFF -> volta a puxar automaticamente dos waypoints)
                        nav_state["vnav_custom_set"] = False
                        wps = nav_state.get("recorded_waypoints", [])
                        idx = nav_state.get("active_waypoint_index", 0)
                        if wps and 0 <= idx < len(wps):
                            nav_state["vnav_target_alt"] = wps[idx].get("alt")
                            vnav_alt_text = str(int(wps[idx].get("alt", 0))) if wps[idx].get("alt") else ""
                        else:
                            nav_state["vnav_target_alt"] = None
                            vnav_alt_text = ""
                        print("Altitude Alvo VNAV restaurada para o modo automático dos waypoints (OFF)")
                    active_input = "none"

                # Clique no botão TOGGLE VNAV TOL (ON/OFF)
                if hasattr(gui, 'btn_set_vnav_tol') and gui.btn_set_vnav_tol.collidepoint(mouse_pos):
                    is_tol_on = nav_state.get("vnav_tol_custom", False)
                    if not is_tol_on:
                        # Liga variação personalizada (ON)
                        try:
                            tol_val = float(vnav_tol_text) if vnav_tol_text else 200.0
                            if tol_val <= 0:
                                tol_val = 200.0
                            nav_state["vnav_tolerance"] = tol_val
                            nav_state["vnav_tol_custom"] = True
                            vnav_tol_text = str(int(tol_val))
                            print(f"Variação Aceita personalizada (ON): +/- {tol_val} FT")
                        except ValueError:
                            print("Tolerância VNAV inválida.")
                    else:
                        # Desliga variação personalizada (OFF -> mantém sempre 200 FT)
                        nav_state["vnav_tolerance"] = 200.0
                        nav_state["vnav_tol_custom"] = False
                        vnav_tol_text = "200"
                        print("Variação Aceita restaurada para o padrão (OFF): +/- 200 FT")
                    active_input = "none"

                # Clique no botão TOGGLE VNAV CAL
                if hasattr(gui, 'btn_toggle_vnav_cal') and gui.btn_toggle_vnav_cal.collidepoint(mouse_pos):
                    is_currently_on = nav_state.get("alt_calibrated", False)
                    if not is_currently_on:
                        # Liga a calibração (ON)
                        try:
                            raw_alt = source.get_data().get("alt", 0)
                            if vnav_cal_text:
                                desired_alt = float(vnav_cal_text)
                            else:
                                # Se a caixa estiver vazia, usa a elevação da pista se existir, senão usa a altitude atual
                                if gs_rwy_alt_text:
                                    desired_alt = float(gs_rwy_alt_text)
                                elif nav_state.get("gs_rwy_elev"):
                                    desired_alt = float(nav_state["gs_rwy_elev"])
                                else:
                                    desired_alt = float(raw_alt)
                                vnav_cal_text = str(int(desired_alt))
                            offset = desired_alt - raw_alt
                            nav_state["alt_offset"] = offset
                            nav_state["alt_calibrated"] = True
                            print(f"Altitude Calibrada (ON)! Desejada: {desired_alt} | Raw: {raw_alt} | Offset: {offset}")
                        except ValueError:
                            print("Valor de calibração inválido.")
                    else:
                        # Desliga a calibração (OFF)
                        nav_state["alt_calibrated"] = False
                        nav_state["alt_offset"] = 0.0
                        print("Calibração de Altitude (Offset): DESLIGADO (OFF)")
                    active_input = "none"

                # Clique no botão LIMPAR RAMPA
                if hasattr(gui, 'btn_clear_gs') and gui.btn_clear_gs and gui.btn_clear_gs.collidepoint(mouse_pos):
                    if nav_state.get("saved_gs_approach") is not None:
                        nav_state["saved_gs_approach"] = None
                        if any(w.get("name", "").startswith("GS") for w in nav_state.get("recorded_waypoints", [])):
                            nav_state["recorded_waypoints"] = []
                            wp_counter = 0
                            course_text = ""
                            vnav_alt_text = ""
                        print("Rampa salva LIMPA da memória com sucesso.")
                    else:
                        print("Nenhuma rampa salva para limpar.")

                # Clique no botão SALVAR/CARREGAR RAMPA
                if hasattr(gui, 'btn_save_gs') and gui.btn_save_gs and gui.btn_save_gs.collidepoint(mouse_pos):
                    if nav_state.get("saved_gs_approach") is not None:
                        # CARREGAR RAMPA
                        backup = nav_state["saved_gs_approach"]
                        nav_state["gs_synthetic_active"] = True
                        nav_state["gs_rwy_elev"] = backup["gs_rwy_elev"]
                        nav_state["gs_thr_lat"] = backup["gs_thr_lat"]
                        nav_state["gs_thr_lon"] = backup["gs_thr_lon"]
                        nav_state["gs_inbound_course"] = backup["gs_inbound_course"]
                        nav_state["gs_inbound_course_true"] = backup["gs_inbound_course_true"]
                        # O curso é SEMPRE o da pista (eixo sagrado) e a origem do XTK é a cabeceira
                        nav_state["course"] = backup["gs_inbound_course"]
                        nav_state["course_true"] = backup["gs_inbound_course_true"]
                        nav_state["lat_orig"] = backup["gs_thr_lat"]
                        nav_state["lon_orig"] = backup["gs_thr_lon"]
                        course_text = f"{int(nav_state['course']):03}"
                        nav_state["recorded_waypoints"] = list(backup["recorded_waypoints"])
                        nav_state["navigation_mode"] = "route"
                        nav_state["active_procedure"] = None
                        nav_state["active_waypoint_index"] = 0
                        nav_state["active"] = True
                        nav_state["system_on"] = True
                        nav_state["missed_approach_climbing"] = False
                        nav_state["prev_dist_to_wp"] = None
                        first_gs_wp = nav_state["recorded_waypoints"][0]
                        nav_state["vnav_target_alt"] = first_gs_wp.get("alt")
                        vnav_alt_text = str(int(first_gs_wp.get("alt", 0))) if first_gs_wp.get("alt") else ""

                        # Ao carregar a rampa salva, permite que o piloto selecione o lado do circuito
                        # livremente via botões contextuais no Card IA, ou detecte pela primeira curva/través.
                        nav_state["circuit_side"] = None
                        nav_state["circuit_side_locked"] = False
                        nav_state["gs_takeoff_active"] = False  # É aproximação/retorno, não decolagem inicial
                        nav_state["is_return_approach"] = True  # Retorno para o campo: ativa botões e direct-to (>4 NM)
                        nav_state["direct_to_airport"] = False

                        # Inicia a vetoração automática identificando posição
                        nav_state["vectoring_phase"] = 1
                        nav_state["vectoring_initial_heading"] = data.get("heading", 0.0)
                        nav_state["pattern_course"] = backup["gs_inbound_course"]
                        nav_state["longitudinal_from_origin"] = None
                        nav_state["abeam_passed"] = False

                        wp_counter = 10
                        recip_hdg = int((backup["gs_inbound_course"] + 180) % 360)
                        print(f"Rampa salva CARREGADA! Proa paralela {recip_hdg:03}°. Escolha o circuito nos botões do Card IA.")
                    else:
                        # SALVAR RAMPA (se estiver ativa)
                        if nav_state.get("gs_synthetic_active") and nav_state.get("recorded_waypoints"):
                            nav_state["saved_gs_approach"] = {
                                "gs_rwy_elev": nav_state["gs_rwy_elev"],
                                "gs_thr_lat": nav_state["gs_thr_lat"],
                                "gs_thr_lon": nav_state["gs_thr_lon"],
                                "gs_inbound_course": nav_state["gs_inbound_course"],
                                "gs_inbound_course_true": nav_state["gs_inbound_course_true"],
                                # Lado do circuito (None se ainda não definido: será gravado assim que o piloto fizer a 1ª curva)
                                "circuit_side": nav_state.get("circuit_side"),
                                "recorded_waypoints": list(nav_state["recorded_waypoints"])
                            }
                            # Mantém o sistema 100% ativo e armado para a decolagem imediata
                            nav_state["system_on"] = True
                            nav_state["active"] = True
                            nav_state["navigation_mode"] = "route"
                            nav_state["active_waypoint_index"] = 0
                            nav_state["gs_takeoff_active"] = True
                            nav_state["is_return_approach"] = False
                            nav_state["direct_to_airport"] = False
                            nav_state["vectoring_phase"] = 1
                            print("Rampa atual SALVA com sucesso em memória. Sistema armado e ativo para a decolagem!")
                        else:
                            print("Nenhuma rampa ativa para salvar.")

                # Clique no botão INICIAR 50FT (Glide Slope Sintético)
                if hasattr(gui, 'btn_start_gs') and gui.btn_start_gs.collidepoint(mouse_pos):
                    # Toggle: se já estiver ativo, desativa
                    if nav_state.get("gs_synthetic_active"):
                        nav_state["gs_synthetic_active"] = False
                        nav_state["gs_rwy_elev"] = None
                        nav_state["gs_thr_lat"] = None
                        nav_state["gs_thr_lon"] = None
                        nav_state["gs_dist_to_threshold"] = None
                        nav_state["circuit_side"] = None
                        nav_state["circuit_side_locked"] = False
                        nav_state["is_return_approach"] = False
                        nav_state["gs_takeoff_active"] = False
                        nav_state["direct_to_airport"] = False
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
                                nav_state["vectoring_phase"] = 1  # Aguardando decolagem / curva de saída (paralela)
                                nav_state["circuit_side"] = None
                                nav_state["circuit_side_locked"] = False
                                nav_state["downwind_established"] = False
                                nav_state["gs_takeoff_active"] = True  # Marca que a aeronave está no chão para decolar!
                                nav_state["is_return_approach"] = False
                                nav_state["direct_to_airport"] = False
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

                # Clique no botão LOG (liga/desliga o monitoramento de curso em tempo real na final)
            # Botao LOG (inicia ou pausa a gravacao de dados de voo no arquivo CSV)
                if gui.btn_log.collidepoint(mouse_pos):
                    nav_state["log_mode"] = not nav_state.get("log_mode", False)
                    nav_state["log_correcting"] = False
                    print(f"Modo LOG: {'LIGADO' if nav_state['log_mode'] else 'DESLIGADO'}")
                    active_input = "none"

                # Clique no botão INICIAR/PARAR MONITOR
            # Botao INICIAR / PARAR MONITORAMENTO LATERAL (ativa travamento de curso e diretor de voo)
                if gui.btn_monitor.collidepoint(mouse_pos):
                    if nav_state["system_on"]:
                        # Parar monitor desliga o sistema, limpa a rampa, desativa os 50 pés e cancela rotas
                        nav_state["system_on"] = False
                        nav_state["active"] = False
                        nav_state["navigation_mode"] = "manual"
                        nav_state["active_waypoint_index"] = 0
                        nav_state["vectoring_phase"] = 0
                        nav_state["log_mode"] = False
                        nav_state["log_correcting"] = False
                        nav_state["log_intercepting"] = False
                        nav_state["log_final_active"] = False

                        # Limpa a rampa salva da memória (faz sumir LIMPAR RAMPA e CARREGAR RAMPA)
                        nav_state["saved_gs_approach"] = None

                        # Desativa os 50 pés (Glide Slope Sintético) e reseta todos os seus parâmetros
                        nav_state["gs_synthetic_active"] = False
                        nav_state["gs_rwy_elev"] = None
                        nav_state["gs_thr_lat"] = None
                        nav_state["gs_thr_lon"] = None
                        nav_state["gs_dist_to_threshold"] = None
                        nav_state["circuit_side"] = None
                        nav_state["circuit_side_locked"] = False
                        nav_state["base_turn_started"] = False
                        nav_state["is_go_around"] = False
                        nav_state["is_return_approach"] = False
                        nav_state["gs_takeoff_active"] = False
                        nav_state["direct_to_airport"] = False
                        nav_state["vnav_gs_alt"] = None
                        nav_state["vnav_target_alt"] = None
                        nav_state["longitudinal_from_origin"] = None
                        nav_state["prev_dist_to_wp"] = None
                        nav_state["downwind_established"] = False

                        # Se houver waypoints da rampa GS gravados, limpa-os da rota
                        if any(w.get("name", "").startswith("GS") for w in nav_state.get("recorded_waypoints", [])):
                            nav_state["recorded_waypoints"] = []
                            wp_counter = 0
                            course_text = ""
                            vnav_alt_text = ""

                        active_input = "none"
                        print("Monitor parado: Sistema desligado, rampa limpa, 50FT desativado e botões resetados.")
                    else:
                        if data.get("connected"):
                            nav_state["system_on"] = True
                            print("Sistema Iniciado! Aguardando inserção de curso manual ou ativação de rota.")

                # Clique no botão GRAVAR WAYPOINT
            # Botao GRAVAR WAYPOINT (adiciona as coordenadas GPS atuais como novo fixo na rota)
                if gui.btn_record_wp.collidepoint(mouse_pos):
                    wps = nav_state["recorded_waypoints"]
                    is_real_sensor = (mode_chosen == "pi") or data.get("is_real_pi", False)
                    sats = data.get("satellites", 0)
                    qual = data.get("gps_quality", 0)
                    has_gps_fix = data.get("gps_fix", True)
                    if is_real_sensor:
                        has_gps_fix = (sats >= 4 and qual > 0)

                    if not data.get("connected"):
                        print("[WAYPOINT] Sem conexão de sensores: impossível gravar waypoint.")
                        nav_state["overshoot_from"] = None
                        nav_state["overshoot_to"] = None
                        nav_state["overshoot_banner"] = "ERRO: SENSORES DESCONECTADOS! Impossível gravar waypoint."
                        nav_state["overshoot_time"] = time.time()
                    elif not has_gps_fix:
                        print(f"[GPS] Bloqueio de segurança: Impossível gravar waypoint sem Fix 3D (Satélites: {sats}).")
                        nav_state["overshoot_from"] = None
                        nav_state["overshoot_to"] = None
                        nav_state["overshoot_banner"] = f"SEM SINAL GPS (SATS: {sats}). AGUARDE FIX 3D PARA GRAVAR WYP!"
                        nav_state["overshoot_time"] = time.time()
                    elif len(wps) < 10:
                        wp_counter += 1
                        wp_name = f"WYP{wp_counter:02d}"
                        mag_var = data.get("mag_var", 0.0)
                        # Usa o curso atual: se há curso SET usa-o; senão calcula pelo deslocamento entre WPs ou track/heading
                        if nav_state["active"]:
                            cur_course = nav_state["course"]
                            cur_course_true = nav_state["course_true"]
                        elif len(wps) >= 1:
                            prev_wp = wps[-1]
                            dist_prev = NavMath.haversine_distance_nm(prev_wp["lat"], prev_wp["lon"], data["lat"], data["lon"])
                            if dist_prev >= 0.005:
                                cur_course_true = NavMath.bearing_degrees(prev_wp["lat"], prev_wp["lon"], data["lat"], data["lon"])
                                cur_course = (cur_course_true - mag_var + 360.0) % 360.0
                                if len(wps) == 1:
                                    # Alinha o eixo de WYP01 exatamente com a perna WYP01 -> WYP02
                                    wps[0]["course_true"] = cur_course_true
                                    wps[0]["course"] = cur_course
                            else:
                                cur_course = data["track"] if data.get("speed", 0.0) >= 3.0 else data["heading"]
                                cur_course_true = (cur_course + mag_var + 360.0) % 360.0
                        else:
                            cur_course = data["track"] if data.get("speed", 0.0) >= 3.0 else data["heading"]
                            cur_course_true = (cur_course + mag_var + 360.0) % 360.0
                        new_wp = {
                            "name": wp_name,
                            "lat": data["lat"],
                            "lon": data["lon"],
                            "alt": data.get("alt", 0),
                            "course": cur_course,
                            "course_true": cur_course_true
                        }
                        wps.append(new_wp)
                        print(f"[WAYPOINT] Gravado: {wp_name} | Lat={data['lat']:.5f} Lon={data['lon']:.5f} | Curso={int(cur_course):03}°")
                        nav_state["overshoot_from"] = None
                        nav_state["overshoot_to"] = None
                        nav_state["overshoot_banner"] = f"WAYPOINT GRAVADO: {wp_name} (Sats: {sats})"
                        nav_state["overshoot_time"] = time.time()
                    else:
                        print("Limite de 10 waypoints atingido.")
                        nav_state["overshoot_from"] = None
                        nav_state["overshoot_to"] = None
                        nav_state["overshoot_banner"] = "LIMITE ATINGIDO: Máximo de 10 waypoints gravados."
                        nav_state["overshoot_time"] = time.time()

                # Clique no botão NAVEGAR WAYPOINTS / REINICIAR WAYPOINTS
            # Botao NAVEGAR/RETORNAR WP (engaja a navegacao para o primeiro waypoint ou reinicia a rota)
                if gui.btn_return_wp.collidepoint(mouse_pos):
                    wps = nav_state["recorded_waypoints"]
                    is_real_sensor = (mode_chosen == "pi") or data.get("is_real_pi", False)
                    sats = data.get("satellites", 0)
                    qual = data.get("gps_quality", 0)
                    has_gps_fix = data.get("gps_fix", True)
                    if is_real_sensor:
                        has_gps_fix = (sats >= 4 and qual > 0)

                    if len(wps) > 0 and data.get("connected"):
                        if not has_gps_fix:
                            print(f"[GPS] Tentativa de navegar rota sem Fix 3D (Satélites: {sats}).")
                            nav_state["overshoot_from"] = None
                            nav_state["overshoot_to"] = None
                            nav_state["overshoot_banner"] = f"SEM SINAL GPS (SATS: {sats}). AGUARDE FIX 3D PARA NAVEGAR!"
                            nav_state["overshoot_time"] = time.time()
                        else:
                            # Se a rampa 50FT estiver ativa, ela continua ativa: o curso da pista (eixo) é sagrado
                            nav_state["navigation_mode"] = "route"
                            nav_state["active_waypoint_index"] = 0
                            nav_state["active"] = True
                            nav_state["system_on"] = True
                            nav_state["circuit_side"] = None
                            nav_state["circuit_side_locked"] = False
                            nav_state["base_turn_started"] = False
                            nav_state["is_go_around"] = False
                            nav_state["circuit_reset_pending"] = True
                            nav_state["is_return_approach"] = False
                            nav_state["gs_takeoff_active"] = bool(nav_state.get("gs_synthetic_active") and data.get("speed", 0.0) < 35.0)
                            nav_state["direct_to_airport"] = False
                            nav_state["prev_dist_to_wp"] = None
                            nav_state["last_wp_advance_time"] = time.time()
                        
                        first_wp = wps[0]
                        
                        if nav_state.get("gs_synthetic_active"):
                            # No modo GS Sintético, não fazemos Direct-To! O eixo da pista é sagrado.
                            nav_state["lat_orig"] = nav_state.get("gs_thr_lat", nav_state["lat_orig"])
                            nav_state["lon_orig"] = nav_state.get("gs_thr_lon", nav_state["lon_orig"])
                            nav_state["course"] = nav_state.get("gs_inbound_course", nav_state["course"])
                            nav_state["course_true"] = nav_state.get("gs_inbound_course_true", nav_state["course_true"])
                            course_text = f"{int(nav_state['course']):03}"
                            nav_state["vnav_target_alt"] = first_wp.get("alt")
                            nav_state["vnav_custom_set"] = False
                            vnav_alt_text = str(int(first_wp.get("alt", 0))) if first_wp.get("alt") else ""
                            nav_state["vectoring_phase"] = 1
                            nav_state["vectoring_initial_heading"] = data["heading"]
                            nav_state["pattern_course"] = first_wp["course"]
                            nav_state["circuit_side"] = None
                            nav_state["circuit_side_locked"] = False
                            nav_state["base_turn_started"] = False
                            nav_state["longitudinal_from_origin"] = None
                            nav_state["downwind_established"] = False
                            nav_state["abeam_passed"] = False
                            print("[50FT GS] Modo rampa sintética iniciado na cabeceira.")
                        elif nav_state.get("active_procedure"):
                            # Procedimentos formais IFR (ex: SBSJ)
                            bearing_true = NavMath.bearing_degrees(data["lat"], data["lon"], first_wp["lat"], first_wp["lon"])
                            mag_var = data.get("mag_var", 0.0)
                            bearing_mag = (bearing_true - mag_var + 360) % 360
                            nav_state["course_true"] = bearing_true
                            nav_state["course"] = bearing_mag
                            nav_state["lat_orig"] = data["lat"]
                            nav_state["lon_orig"] = data["lon"]
                            course_text = f"{int(bearing_mag):03}"
                            nav_state["vnav_target_alt"] = first_wp.get("alt")
                            nav_state["vnav_custom_set"] = False
                            vnav_alt_text = str(int(first_wp.get("alt", 0))) if first_wp.get("alt") else ""
                            nav_state["vectoring_phase"] = 0
                            print(f"[PROCEDIMENTO] Rota IFR iniciada direto para {first_wp['name']}.")
                        else:
                            # ── NAVEGAÇÃO CONVENCIONAL DE WAYPOINTS (Manual) ──
                            prev_mode = nav_state.get("navigation_mode")

                            # Se a rota foi concluída (passou todos os WPs) ou já estava navegando e quer retornar ao WP01:
                            if prev_mode in ("route", "route_finished") or len(wps) == 1:
                                first_wp = wps[0]
                                nav_state["navigation_mode"] = "route"
                                nav_state["active_waypoint_index"] = 0
                                nav_state["active"] = True
                                nav_state["system_on"] = True
                                nav_state["lat_orig"] = first_wp["lat"]
                                nav_state["lon_orig"] = first_wp["lon"]
                                nav_state["course"] = first_wp["course"]
                                nav_state["course_true"] = first_wp["course_true"]
                                course_text = f"{int(first_wp['course']):03}"
                                nav_state["pattern_course"] = first_wp["course"]
                                nav_state["vnav_target_alt"] = first_wp.get("alt")
                                nav_state["vnav_custom_set"] = False
                                vnav_alt_text = str(int(first_wp.get("alt", 0))) if first_wp.get("alt") else ""

                                # Inicia o circuito de afastamento e aproximação ao WP01:
                                nav_state["vectoring_phase"] = 1
                                nav_state["vectoring_initial_heading"] = data["heading"]
                                nav_state["circuit_side"] = None
                                nav_state["circuit_side_locked"] = False
                                nav_state["downwind_established"] = False
                                nav_state["abeam_passed"] = False
                                nav_state["base_turn_started"] = False
                                nav_state["longitudinal_from_origin"] = None
                                nav_state["prev_dist_to_wp"] = None
                                nav_state["last_wp_advance_time"] = time.time()
                                nav_state["overshoot_from"] = None
                                nav_state["overshoot_to"] = None
                                nav_state["overshoot_banner"] = f"RETORNO AO WP01 INICIADO: Afaste 1 NM apos o traves para curva base."
                                nav_state["overshoot_time"] = time.time()
                                print(f"[WAYPOINTS] Retorno ao WP01 iniciado. Circuito ativo (Eixo {int(first_wp['course']):03}°).")
                            else:
                                # Início inicial da rota (WP01 -> WP02 -> ...)
                                nav_state["navigation_mode"] = "route"
                                nav_state["active"] = True
                                nav_state["system_on"] = True
                                nav_state["circuit_side"] = None
                                nav_state["circuit_side_locked"] = False
                                nav_state["base_turn_started"] = False
                                nav_state["longitudinal_from_origin"] = None
                                nav_state["downwind_established"] = False
                                nav_state["abeam_passed"] = False
                                nav_state["vectoring_phase"] = 0
                                mag_var = data.get("mag_var", 0.0)

                                if len(wps) >= 2:
                                    dist_to_w0 = NavMath.haversine_distance_nm(data["lat"], data["lon"], wps[0]["lat"], wps[0]["lon"])
                                    if dist_to_w0 <= 0.08:
                                        active_idx = 1
                                        orig_lat, orig_lon = wps[0]["lat"], wps[0]["lon"]
                                        tgt_wp = wps[1]
                                        leg_brg_true = NavMath.bearing_degrees(wps[0]["lat"], wps[0]["lon"], wps[1]["lat"], wps[1]["lon"])
                                    else:
                                        active_idx = 0
                                        orig_lat, orig_lon = data["lat"], data["lon"]
                                        tgt_wp = wps[0]
                                        leg_brg_true = NavMath.bearing_degrees(data["lat"], data["lon"], wps[0]["lat"], wps[0]["lon"])

                                    nav_state["active_waypoint_index"] = active_idx
                                    nav_state["lat_orig"] = orig_lat
                                    nav_state["lon_orig"] = orig_lon
                                    leg_brg_mag = (leg_brg_true - mag_var + 360) % 360
                                    nav_state["course_true"] = leg_brg_true
                                    nav_state["course"] = leg_brg_mag
                                    course_text = f"{int(leg_brg_mag):03}"
                                    nav_state["vnav_target_alt"] = tgt_wp.get("alt")
                                    nav_state["vnav_custom_set"] = False
                                    vnav_alt_text = str(int(tgt_wp.get("alt", 0))) if tgt_wp.get("alt") else ""
                                    nav_state["overshoot_from"] = None
                                    nav_state["overshoot_to"] = None
                                    nav_state["overshoot_banner"] = f"ROTA INICIADA: Perna para {tgt_wp['name']} (Curso {int(leg_brg_mag):03}°)"
                                    nav_state["overshoot_time"] = time.time()
                                    print(f"[WAYPOINTS] Navegação convencional: perna para {tgt_wp['name']} (Curso: {int(leg_brg_mag):03}°).")
                                else:
                                    nav_state["active_waypoint_index"] = 0
                                    nav_state["lat_orig"] = data["lat"]
                                    nav_state["lon_orig"] = data["lon"]
                                    leg_brg_true = NavMath.bearing_degrees(data["lat"], data["lon"], wps[0]["lat"], wps[0]["lon"])
                                    leg_brg_mag = (leg_brg_true - mag_var + 360) % 360
                                    nav_state["course_true"] = leg_brg_true
                                    nav_state["course"] = leg_brg_mag
                                    course_text = f"{int(leg_brg_mag):03}"
                                    nav_state["vnav_target_alt"] = wps[0].get("alt")
                                    nav_state["vnav_custom_set"] = False
                                    vnav_alt_text = str(int(wps[0].get("alt", 0))) if wps[0].get("alt") else ""
                                    print(f"[WAYPOINTS] Direct-To para {wps[0]['name']} (Curso: {int(leg_brg_mag):03}°).")
                    else:
                        print("Nenhum waypoint gravado para navegar.")

                # Clique no botão ZERAR WAYPOINTS
            # Botao ZERAR WAYPOINTS (apaga todos os fixos da rota personalizada)
                if gui.btn_clear_wps.collidepoint(mouse_pos):
                    if nav_state["recorded_waypoints"] or nav_state.get("gs_synthetic_active") or nav_state.get("vnav_target_alt") is not None or nav_state.get("vnav_gs_alt") is not None:
                        nav_state["recorded_waypoints"] = []
                        nav_state["gs_synthetic_active"] = False
                        nav_state["gs_inbound_course"] = None
                        nav_state["gs_inbound_course_true"] = None
                        nav_state["navigation_mode"] = "manual"
                        nav_state["active_waypoint_index"] = 0
                        nav_state["active"] = False
                        nav_state["vectoring_phase"] = 0
                        nav_state["circuit_side"] = None
                        nav_state["circuit_side_locked"] = False
                        nav_state["base_turn_started"] = False
                        nav_state["is_return_approach"] = False
                        nav_state["gs_takeoff_active"] = False
                        nav_state["overshoot_from"] = None
                        nav_state["overshoot_to"] = None
                        nav_state["direct_to_airport"] = False
                        nav_state["vnav_target_alt"] = None
                        nav_state["vnav_gs_alt"] = None
                        nav_state["vnav_custom_set"] = False
                        nav_state["active_procedure"] = None
                        nav_state["gs_rwy_elev"] = None
                        nav_state["gs_thr_lat"] = None
                        nav_state["gs_thr_lon"] = None
                        nav_state["gs_dist_to_threshold"] = None
                        nav_state["overshoot_banner"] = None
                        nav_state["overshoot_time"] = 0
                        wp_counter = 0
                        course_text = ""
                        vnav_alt_text = ""
                        print("Todos os waypoints zerados e alertas de altitude limpos.")
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
                                nav_state["circuit_side_locked"] = False
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
                    # (Escala lateral esquerda de waypoints é apenas visual - clique desabilitado)

                    # Clique no botão PRÓXIMO WAYPOINT (avança WYP01->WYP02->WYP03 mantendo o trilho original gravado)
                    if getattr(gui, 'btn_next_wp', None) and gui.btn_next_wp.collidepoint(mouse_pos):
                        wps = nav_state.get("recorded_waypoints", [])
                        idx = nav_state.get("active_waypoint_index", 0)
                        if (nav_state.get("active") and nav_state.get("navigation_mode") == "route"
                                and not nav_state.get("gs_synthetic_active")
                                and not nav_state.get("active_procedure")
                                and 0 <= idx < len(wps) - 1):
                            curr_wp = wps[idx]
                            next_idx = idx + 1
                            next_wp = wps[next_idx]
                            nav_state["active_waypoint_index"] = next_idx
                            nav_state["last_wp_advance_time"] = time.time()
                            nav_state["circuit_side"] = None
                            nav_state["circuit_side_locked"] = False
                            nav_state["downwind_established"] = False
                            nav_state["abeam_passed"] = False
                            nav_state["base_turn_started"] = False
                            nav_state["longitudinal_from_origin"] = None
                            nav_state["prev_dist_to_wp"] = None
                            nav_state["vectoring_phase"] = 0
                            # Mantém o trilho original gravado entre curr_wp e next_wp (NÃO recalcula da posição atual)
                            nav_state["lat_orig"] = curr_wp["lat"]
                            nav_state["lon_orig"] = curr_wp["lon"]
                            leg_brg_true = NavMath.bearing_degrees(curr_wp["lat"], curr_wp["lon"], next_wp["lat"], next_wp["lon"])
                            mag_var = data.get("mag_var", 0.0)
                            leg_brg_mag = (leg_brg_true - mag_var + 360) % 360
                            nav_state["course_true"] = leg_brg_true
                            nav_state["course"] = leg_brg_mag
                            course_text = f"{int(leg_brg_mag):03}"
                            if not nav_state.get("vnav_custom_set"):
                                nav_state["vnav_target_alt"] = next_wp.get("alt")
                                vnav_alt_text = str(int(next_wp.get("alt", 0))) if next_wp.get("alt") else ""
                            nav_state["overshoot_from"] = None
                            nav_state["overshoot_to"] = None
                            nav_state["overshoot_banner"] = f"AVANÇO MANUAL: Perna {curr_wp['name']} -> {next_wp['name']} (Trilho original {int(leg_brg_mag):03}°)"
                            nav_state["overshoot_time"] = time.time()
                            print(f"[MANUAL] Avanço manual: {curr_wp['name']} -> {next_wp['name']} mantendo trilho original (Curso: {int(leg_brg_mag):03}°).")

                    # Clique nos botões contextuais de seleção de lado do circuito (Card IA)
                    if getattr(gui, 'btn_side_left', None) and gui.btn_side_left.collidepoint(mouse_pos):
                        nav_state["circuit_side"] = "L"
                        nav_state["circuit_side_locked"] = True
                        nav_state["circuit_reset_pending"] = False
                        nav_state["direct_to_airport"] = False  # Cancela o Direct-To imediatamente!
                        nav_state["is_return_approach"] = False # Finaliza a seleção de retorno
                        nav_state["gs_takeoff_active"] = False
                        if nav_state.get("saved_gs_approach") is not None:
                            nav_state["saved_gs_approach"]["circuit_side"] = "L"
                        print("Piloto selecionou/trocou: CIRCUITO ESQUERDA (L). Travado no lado escolhido.")

                    elif getattr(gui, 'btn_side_right', None) and gui.btn_side_right.collidepoint(mouse_pos):
                        nav_state["circuit_side"] = "R"
                        nav_state["circuit_side_locked"] = True
                        nav_state["circuit_reset_pending"] = False
                        nav_state["direct_to_airport"] = False  # Cancela o Direct-To imediatamente!
                        nav_state["is_return_approach"] = False # Finaliza a seleção de retorno
                        nav_state["gs_takeoff_active"] = False
                        if nav_state.get("saved_gs_approach") is not None:
                            nav_state["saved_gs_approach"]["circuit_side"] = "R"
                        print("Piloto selecionou/trocou: CIRCUITO DIREITA (R). Travado no lado escolhido.")

                    # Clique no botão DIR->
                    if hasattr(gui, 'btn_dir') and gui.btn_dir and gui.btn_dir.collidepoint(mouse_pos):
                        show_dir_dropdown = not show_dir_dropdown
                    
                    # Clique no botão TOGA
                    if hasattr(gui, 'btn_toga') and gui.btn_toga and gui.btn_toga.collidepoint(mouse_pos):
                        wps = nav_state["recorded_waypoints"]
                        if wps and nav_state.get("active_procedure"):
                            print("*** ARREMETIDA MANUAL (TOGA)! Redirecionando para MAHF ***")
                            nav_state["circuit_side"] = None
                            nav_state["circuit_side_locked"] = False
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
                        elif wps and nav_state.get("gs_synthetic_active"):
                            print("*** ARREMETIDA MANUAL (TOGA)! Reiniciando circuito de 50FT ***")
                            nav_state["is_go_around"] = True
                            nav_state["circuit_side"] = None
                            nav_state["circuit_side_locked"] = False
                            nav_state["vectoring_phase"] = 1
                            nav_state["gs_takeoff_active"] = True
                            nav_state["active_waypoint_index"] = 0
                            nav_state["abeam_passed"] = False
                            nav_state["navigation_mode"] = "route"
                            nav_state["active"] = True
                            continue

                    # Clique no botão CYCLE CHART (Abre o dropdown)
                    if hasattr(gui, 'btn_cycle_chart') and gui.btn_cycle_chart.collidepoint(mouse_pos):
                        show_chart_dropdown = not show_chart_dropdown

                    if hasattr(gui, 'btn_load_chart') and gui.btn_load_chart.collidepoint(mouse_pos) and selected_chart_name:
                        nav_state["gs_synthetic_active"] = False
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
                        nav_state["circuit_side_locked"] = False
                        nav_state["is_return_approach"] = False
                        nav_state["gs_takeoff_active"] = False
                        nav_state["direct_to_airport"] = False
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
                            if tol_val <= 0:
                                tol_val = 10.0
                                vnav_tol_text = "10"
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
                        # Permite sinal negativo apenas no início e dígitos
                        ch = event.unicode
                        if ch.isdigit() and len(vnav_cal_text) < 6:
                            vnav_cal_text += ch
                        elif ch == "-" and len(vnav_cal_text) == 0:
                            vnav_cal_text += ch

                elif active_input == "gs_rwy_alt":
                    # Campo ALTITUDE RWY: aceita apenas dígitos inteiros (altitude do aeródromo)
                    if event.key == pygame.K_RETURN:
                        active_input = "none"
                    elif event.key == pygame.K_BACKSPACE:
                        gs_rwy_alt_text = gs_rwy_alt_text[:-1]
                    else:
                        if event.unicode.isdigit() and len(gs_rwy_alt_text) < 5:
                            gs_rwy_alt_text += event.unicode

                elif active_input == "cal_hdg":
                    if event.key == pygame.K_RETURN:
                        try:
                            cal_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sensor_calibration.json")
                            calib = {}
                            if os.path.exists(cal_path):
                                try:
                                    with open(cal_path, "r", encoding="utf-8") as f:
                                        calib = json.load(f)
                                except Exception:
                                    pass
                            if hdg_cal_text in ("reset", "ZERAR", "zerar"):
                                calib["heading_offset"] = 0.0
                                calib["timestamp"] = time.time()
                                calib["calibrated_via"] = "Reset to Sensor Natural Heading (ZERAR)"
                                with open(cal_path, "w", encoding="utf-8") as f:
                                    json.dump(calib, f, indent=4)
                                if hasattr(source, "calibration"):
                                    source.calibration["heading_offset"] = 0.0
                                print("[OK] Offset de proa resetado para 0.0° (Proa natural do sensor restaurada).")
                            elif hdg_cal_text:
                                desired_hdg = float(hdg_cal_text) % 360.0
                                current_hdg = data.get("heading", 0.0)
                                old_offset = calib.get("heading_offset", 0.0)
                                base_hdg = (current_hdg - old_offset) % 360.0
                                new_offset = (desired_hdg - base_hdg) % 360.0
                                calib["heading_offset"] = new_offset
                                calib["timestamp"] = time.time()
                                calib["calibrated_via"] = f"Manual Keyboard Align ({int(desired_hdg):03}°)"
                                with open(cal_path, "w", encoding="utf-8") as f:
                                    json.dump(calib, f, indent=4)
                                if hasattr(source, "calibration"):
                                    source.calibration["heading_offset"] = new_offset
                                print(f"[OK] Proa Sincronizada: {int(desired_hdg):03} (Offset: {new_offset:+.1f})")
                        except ValueError:
                            pass
                        active_input = "none"
                    elif event.key == pygame.K_BACKSPACE:
                        hdg_cal_text = hdg_cal_text[:-1]
                    else:
                        if event.unicode.isdigit() and len(hdg_cal_text) < 3:
                            hdg_cal_text += event.unicode
                
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

        # Atualiza status global do Co-Piloto Neural a cada iteração (sempre ativo, inclusive em Standby)
        nav_state["ai_online"] = ai_copilot.is_online()
        if not nav_state.get("active") and data.get("connected"):
            ai_copilot.update_telemetry(
                runway_course=nav_state.get("course", 0.0),
                current_heading=data.get("heading", 0.0),
                xtk_nm=nav_state.get("xtk", 0.0),
                dist_nm=nav_state.get("dist_to_wp"),
                ias_kt=data.get("speed", 100.0),
                v_phase=0,
                is_active=False
            )

        # Atualiza a física de navegação se estiver ativa e houver dados válidos
        if nav_state["active"] and data.get("connected"):
            # ── Modo Rota: auto-sequenciamento de waypoints ──────────────────
            if nav_state["navigation_mode"] == "route":
                wps = nav_state["recorded_waypoints"]
                idx = nav_state["active_waypoint_index"]

                if idx < len(wps):
                    target_wp = wps[idx]
                    wp_lat = target_wp["lat"]
                    wp_lon = target_wp["lon"]
                    course_true = target_wp["course_true"]
                    
                    # 1. Distância ao waypoint ativo sempre atualizada em tempo real a cada frame
                    dist = NavMath.haversine_distance_nm(data["lat"], data["lon"], wp_lat, wp_lon)
                    nav_state["dist_to_wp"] = dist

                    v_phase = nav_state.get("vectoring_phase", 0)
                    
                    if v_phase > 0:
                        # ── Nova Lógica Geométrica Contínua em Tempo Real (FMS Vectoring) ──
                        # Diferente de sistemas baseados em cronômetros, este algoritmo utiliza a
                        # geometria espacial contínua para guiar a aeronave em procedimentos (arremetidas, esperas).
                        current_hdg = data["heading"]

                        # Posição Longitudinal em relação à Origem do Procedimento (Cabeceira no GS ou WP01 nos Waypoints)
                        gs_axis_mode = bool(nav_state.get("gs_synthetic_active") and not nav_state.get("active_procedure"))
                        if gs_axis_mode:
                            ref_lat = nav_state.get("gs_thr_lat", wp_lat)
                            ref_lon = nav_state.get("gs_thr_lon", wp_lon)
                            axis_course_true = nav_state.get("gs_inbound_course_true", course_true)
                            axis_course_mag = nav_state.get("gs_inbound_course", target_wp["course"])
                        else:
                            first_wp = wps[0] if wps else target_wp
                            ref_lat = first_wp["lat"]
                            ref_lon = first_wp["lon"]
                            axis_course_true = first_wp["course_true"]
                            axis_course_mag = first_wp["course"]

                        brg_from_ref = NavMath.bearing_degrees(ref_lat, ref_lon, data["lat"], data["lon"])
                        dist_from_ref = NavMath.haversine_distance_nm(ref_lat, ref_lon, data["lat"], data["lon"])
                        
                        # Projeção no eixo do curso (inbound para GS, rumo da rota para waypoints):
                        # Positivo (+) = À FRENTE da referência (setor de decolagem / subida / além da cabeceira)
                        # Negativo (-) = ATRÁS da referência (setor de aproximação / perna do vento após o través)
                        # Zero (0)     = EXATAMENTE no través da referência (Través da cabeceira / Bloqueio do través)
                        angle_diff_ref_rad = math.radians(NavMath.normalize_angle(brg_from_ref - axis_course_true))
                        longitudinal_nm = dist_from_ref * math.cos(angle_diff_ref_rad)
                        if dist_from_ref < 0.03:
                            longitudinal_nm = 0.0
                        nav_state["longitudinal_from_origin"] = longitudinal_nm
                        nav_state["abeam_passed"] = bool(longitudinal_nm is not None and longitudinal_nm <= 0.05)
                        
                        xtk_nm = nav_state.get("xtk", 0.0)

                        # 2. Identificação Dinâmica do Lado do Circuito e Proas Relativas
                        hdg_diff_inbound = abs(NavMath.normalize_angle(current_hdg - axis_course_mag))
                        reciprocal_hdg = (axis_course_mag + 180) % 360
                        hdg_diff_reciprocal = abs(NavMath.normalize_angle(current_hdg - reciprocal_hdg))

                        # Curva em relação ao curso da pista (ex: 262°)
                        turn_from_runway = NavMath.normalize_angle(current_hdg - axis_course_mag)

                        circuit_side = nav_state.get("circuit_side")

                        # A identificação automática de lado SÓ atua se o circuito ainda NÃO tiver um lado definido (circuit_side is None)
                        # e NÃO estiver em aproximação de retorno à distância (onde o piloto escolhe via botões a 4 NM):
                        if not circuit_side and not nav_state.get("is_return_approach", False):
                            side_found = None
                            how_found = ""

                            turn_from_runway = NavMath.normalize_angle(current_hdg - axis_course_mag)
                            vec_init_hdg = nav_state.get("vectoring_initial_heading", axis_course_mag)
                            init_diff_from_axis = abs(NavMath.normalize_angle(vec_init_hdg - axis_course_mag))
                            turn_from_init = NavMath.normalize_angle(current_hdg - vec_init_hdg)
                            has_turned_since_start = (init_diff_from_axis < 25.0) or (abs(turn_from_init) >= 20.0)
                            moving_for_turn = (data.get("speed", 0.0) >= 35.0)
                            min_downwind_xtk = 0.25

                            # 1. Primeira curva deliberada de saída da pista / decolagem (crosswind / perna de través):
                            # Se fez uma curva de 25° a 155° em relação ao eixo da pista em deslocamento:
                            if 25.0 <= abs(turn_from_runway) <= 155.0 and moving_for_turn and has_turned_since_start:
                                turn_ref = turn_from_runway if init_diff_from_axis < 25.0 else turn_from_init
                                side_found = "R" if turn_ref > 0 else "L"
                                how_found = f"1ª curva de saída da pista ({abs(turn_ref):.0f}° para {'DIREITA' if side_found == 'R' else 'ESQUERDA'})"

                            # 2. Se já estiver voando a perna do vento (recíproca, ~082°) com deslocamento lateral nítido:
                            elif hdg_diff_reciprocal <= 35.0 and abs(xtk_nm) > min_downwind_xtk:
                                side_found = "R" if xtk_nm > 0 else "L"
                                how_found = f"voando a perna do vento ({current_hdg:.0f}°), XTK={xtk_nm:.2f} NM ({'DIREITA' if side_found == 'R' else 'ESQUERDA'})"

                            # 3. Se estiver no setor afastado após o través e curvando para a base:
                            elif longitudinal_nm is not None and longitudinal_nm <= -0.25:
                                turn_from_downwind = NavMath.normalize_angle(current_hdg - reciprocal_hdg)
                                if 25.0 <= abs(turn_from_downwind) <= 120.0 and data.get("speed", 0.0) >= 35.0:
                                    side_found = "R" if turn_from_downwind > 0 else "L"
                                    how_found = f"curva base a partir da perna do vento ({abs(turn_from_downwind):.0f}° para {'DIREITA' if side_found == 'R' else 'ESQUERDA'})"

                            # Uma vez identificado pela primeira curva/posição, TRAVA no lado e encerra a adivinhação:
                            if side_found:
                                circuit_side = side_found
                                nav_state["circuit_side"] = circuit_side
                                nav_state["circuit_side_locked"] = True  # TRAVADO! Não oscila nunca mais!
                                nav_state["gs_takeoff_active"] = False
                                nav_state["is_go_around"] = False
                                nav_state["is_return_approach"] = False
                                nav_state["circuit_reset_pending"] = False
                                if gs_axis_mode and nav_state.get("saved_gs_approach") is not None:
                                    nav_state["saved_gs_approach"]["circuit_side"] = circuit_side
                                print(f"Lado do circuito identificado e travado com sucesso: {'DIREITA' if circuit_side == 'R' else 'ESQUERDA'} ({how_found}).")

                        is_go_around = nav_state.get("is_go_around", False)
                        if is_go_around:
                            # Em arremetida: mantém o modo de subida na proa da pista
                            nav_state["gs_takeoff_active"] = True
                            if (longitudinal_nm is not None and longitudinal_nm > 2.0) or circuit_side:
                                nav_state["is_go_around"] = False
                                nav_state["gs_takeoff_active"] = False
                        else:
                            # Se o avião curvou deliberadamente para fora do eixo da pista (>= 25° com velocidade >= 5 kt),
                            # ou ultrapassou 2.5 NM na subida de proa de pista, ou o lado do circuito foi travado:
                            turn_away = (abs(turn_from_runway) >= 25.0 and data.get("speed", 0.0) >= 35.0)
                            dist_away = (longitudinal_nm is not None and longitudinal_nm > 2.5)
                            if turn_away or dist_away or circuit_side:
                                nav_state["gs_takeoff_active"] = False

                        # 3. Classificação Dinâmica da Perna Atual (State Machine real-time)
                        bearing_to_wp = NavMath.bearing_degrees(data["lat"], data["lon"], wp_lat, wp_lon)

                        # Verificação de Aproximação Direta (Inbound):
                        # Só ativa se o avião estiver DE FATO no setor afastado da aproximação final (longitudinal <= -1.0 NM)
                        # e voando com proa orientada para o pouso/inbound (hdg_diff_inbound <= 45°),
                        # e NÃO estiver em decolagem ativa e NÃO estiver em arremetida:
                        is_flying_inbound = (not is_go_around
                                             and not nav_state.get("gs_takeoff_active", False)
                                             and not nav_state.get("is_return_approach", False)
                                             and longitudinal_nm is not None and longitudinal_nm <= -1.0
                                             and hdg_diff_inbound <= 45.0)

                        if is_flying_inbound:
                            if abs(xtk_nm) <= nav_state.get("xtk_tolerance", 0.3) and hdg_diff_inbound <= 25.0:
                                nav_state["vectoring_phase"] = 7  # Final / Inbound estabelecido
                                nav_state["lat_orig"] = wp_lat
                                nav_state["lon_orig"] = wp_lon
                                nav_state["course_true"] = axis_course_true
                                nav_state["course"] = axis_course_mag
                                course_text = f"{int(axis_course_mag):03}"
                                nav_state["direct_bearing"] = bearing_to_wp
                            else:
                                nav_state["vectoring_phase"] = 6  # Interceptação de Final
                                nav_state["lat_orig"] = wp_lat
                                nav_state["lon_orig"] = wp_lon
                                nav_state["course_true"] = axis_course_true
                                nav_state["course"] = axis_course_mag
                                course_text = f"{int(axis_course_mag):03}"
                                nav_state["direct_bearing"] = bearing_to_wp

                        elif not circuit_side:
                            nav_state["vectoring_phase"] = 1  # Fase 1: Identificando lado pela 1ª curva

                        else:
                            # Circuito com lado definido
                            base_hdg = (axis_course_mag + 90) % 360 if circuit_side == "L" else (axis_course_mag - 90 + 360) % 360
                            diff_to_base = abs(NavMath.normalize_angle(current_hdg - base_hdg))
                            dist_past_abeam = abs(longitudinal_nm) if longitudinal_nm is not None else 0.0
                            target_base_dist = 10.0 if gs_axis_mode else 1.0  # GS: 10 NM | WPs Manuais: 1.0 NM

                            # Curva em direção à perna base a partir da perna do vento (recíproca):
                            turn_from_downwind = NavMath.normalize_angle(current_hdg - reciprocal_hdg)
                            if circuit_side == "L":
                                # Curva para a esquerda a partir da perna do vento (-45° a -135° para a base; exclui proa de decolagem ~ -180°)
                                is_curving_to_base_45 = (-135.0 <= turn_from_downwind <= -45.0)
                                base_turn_started = (-135.0 <= turn_from_downwind <= -30.0)
                            else:
                                # Curva para a direita a partir da perna do vento (+45° a +135° para a base; exclui proa de decolagem ~ +180°)
                                is_curving_to_base_45 = (45.0 <= turn_from_downwind <= 135.0)
                                base_turn_started = (30.0 <= turn_from_downwind <= 135.0)

                            # Registro do través
                            if longitudinal_nm is not None and longitudinal_nm <= 0.05:
                                nav_state["abeam_passed"] = True
                            else:
                                nav_state["abeam_passed"] = False

                            # Verifica se o avião já estabeleceu na perna do vento (paralela)
                            # Se voou a proa da perna do vento (diff <= 40°), confirma que ingressou no circuito
                            if hdg_diff_reciprocal <= 40.0:
                                nav_state["downwind_established"] = True

                            downwind_ok = nav_state.get("downwind_established", False)

                            # Antecipação exata aos 45°:
                            # SÓ ativa base deliberada se já ingressou na perna do vento (evita acionar na subida inicial/decolagem).
                            # Se já estabeleceu, ao curvar 45° para o lado ativo OU aproximar da proa da base (<= 45°),
                            # ativa a base imediatamente (inclusive antes do través / base antecipada).
                            # Se não curvou, ativa automaticamente ao atingir target_base_dist após o través:
                            is_turning_base = downwind_ok and (is_curving_to_base_45 or diff_to_base <= 45.0)
                            if nav_state["abeam_passed"] and dist_past_abeam >= target_base_dist:
                                is_turning_base = True

                            nav_state["base_turn_started"] = base_turn_started

                            # Verificação de curva de 45° a partir da perna do vento em direção à base:
                            if circuit_side == "L":
                                is_curved_base_45 = (-135.0 <= turn_from_downwind <= -45.0) or (diff_to_base <= 45.0)
                            else:
                                is_curved_base_45 = (45.0 <= turn_from_downwind <= 135.0) or (diff_to_base <= 45.0)

                            # Curva a partir da base em direção à final:
                            turn_from_base = NavMath.normalize_angle(current_hdg - base_hdg)
                            if circuit_side == "L":
                                # Curvou a partir da base em direção à final à esquerda (base 352° -> final 263°)
                                # Reconhece curva a partir de 35° a 45° da base em direção à final
                                is_curving_to_final = (-120.0 <= turn_from_base <= -35.0)
                            else:
                                # Curvou a partir da base em direção à final à direita (base 172° -> final 263°)
                                # Reconhece curva a partir de 35° a 45° da base em direção à final
                                is_curving_to_final = (35.0 <= turn_from_base <= 120.0)

                            # 1. FINAL / INTERCEPTAÇÃO:
                            # SÓ entra em final/interceptação se:
                            # a) Já estiver na interceptação/final (v_phase in (6, 7)), OU
                            # b) Estiver na perna base (v_phase == 5) ou iniciando base confirmada (downwind_ok and is_turning_base), E:
                            #    - Já estiver na base e chegando perto do eixo de interceptação (|XTK| <= intercept_xtk_gate), OU
                            #    - Iniciou curva da base para a final (is_curving_to_final, >= 35°-45°), OU
                            #    - A proa já está orientada para a final (hdg_diff_inbound <= 55.0)
                            intercept_xtk_gate = 0.35
                            in_base_or_final = (v_phase in (5, 6, 7) or (downwind_ok and is_turning_base))
                            is_intercept_trigger = (
                                v_phase in (6, 7) or
                                (is_curved_base_45 and abs(xtk_nm) <= intercept_xtk_gate) or
                                is_curving_to_final or
                                hdg_diff_inbound <= 55.0
                            )
                            if in_base_or_final and is_intercept_trigger:
                                if abs(xtk_nm) <= nav_state.get("xtk_tolerance", 0.3) and hdg_diff_inbound <= 25.0:
                                    nav_state["vectoring_phase"] = 7  # Final / Inbound estabelecido
                                else:
                                    nav_state["vectoring_phase"] = 6  # Interceptação
                                nav_state["lat_orig"] = wp_lat
                                nav_state["lon_orig"] = wp_lon
                                nav_state["course_true"] = axis_course_true
                                nav_state["course"] = axis_course_mag
                                course_text = f"{int(axis_course_mag):03}"
                                nav_state["direct_bearing"] = bearing_to_wp

                            # 2. PERNA BASE:
                            # Se curvou 45° para a base (após perna do vento), ou já estava na base e ainda não interceptou a final:
                            elif is_turning_base or v_phase == 5 or (downwind_ok and diff_to_base <= 30.0):
                                nav_state["vectoring_phase"] = 5  # Perna Base

                            # 3. PERNA PARALELA / DO VENTO:
                            # Durante a decolagem/subida inicial e na perna do vento até a curva base:
                            else:
                                if nav_state["abeam_passed"] and dist_past_abeam > 0.5:
                                    nav_state["vectoring_phase"] = 3  # Perna do vento afastando após o través
                                else:
                                    nav_state["vectoring_phase"] = 2  # Perna paralela de retorno / no través

                    # ── Detecção Automática de Arremetida (Missed Approach na Final) ──
                    # Só atua na aproximação final (< 5.0 NM da pista) com subida deliberada e sustentada:
                    # VSI > 500 ft/min e Pitch > +2.0° por pelo menos 5.0 segundos consecutivos
                    dist_to_runway = nav_state.get("gs_dist_to_threshold") if gs_axis_mode else nav_state.get("dist_to_wp")
                    is_on_short_final = (dist_to_runway is not None and dist_to_runway <= 5.0)

                    if (not nav_state.get("active_procedure")) and nav_state.get("vectoring_phase", 0) in (6, 7) and is_on_short_final and data.get("speed", 0.0) >= 40.0 and not nav_state.get("is_go_around", False):
                        vsi_val = data.get("vsi", 0.0)
                        pitch_val = data.get("pitch", 0.0)
                        if vsi_val > 500.0 and pitch_val > 2.0:
                            climb_t = nav_state.get("go_around_climb_start")
                            if climb_t is None:
                                nav_state["go_around_climb_start"] = time.time()
                            elif time.time() - climb_t >= 5.0:
                                print("*** ARREMETIDA CONFIRMADA (SUBIDA SUSTENTADA > 500 FT/MIN)! Reiniciando circuito ***")
                                nav_state["is_go_around"] = True
                                nav_state["circuit_side"] = None
                                nav_state["circuit_side_locked"] = False
                                nav_state["base_turn_started"] = False
                                nav_state["circuit_reset_pending"] = True
                                nav_state["vectoring_phase"] = 1
                                nav_state["gs_takeoff_active"] = True
                                nav_state["is_return_approach"] = False
                                nav_state["direct_to_airport"] = False
                                nav_state["active_waypoint_index"] = 0
                                nav_state["abeam_passed"] = False
                                nav_state["navigation_mode"] = "route"
                                nav_state["active"] = True
                                nav_state["go_around_climb_start"] = None
                        else:
                            nav_state["go_around_climb_start"] = None
                    else:
                        nav_state["go_around_climb_start"] = None

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
                                nav_state["circuit_side_locked"] = False
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
                                if dist_to_thr >= 9.0:
                                    idx_calc = 0
                                else:
                                    idx_calc = int(9 - math.floor(dist_to_thr))
                                if idx_calc < 0: idx_calc = 0
                                if idx_calc > len(wps) - 1: idx_calc = len(wps) - 1
                                
                                if idx_calc != idx:
                                    nav_state["active_waypoint_index"] = idx_calc
                                    target_wp = wps[idx_calc]
                                    nav_state["lat_orig"] = target_wp["lat"]
                                    nav_state["lon_orig"] = target_wp["lon"]
                                    if not nav_state.get("vnav_custom_set"):
                                        nav_state["vnav_target_alt"] = target_wp.get("alt")
                                        vnav_alt_text = str(int(target_wp.get("alt", 0))) if target_wp.get("alt") else ""
                                    print(f"GS Sintético: Sequenciado para {target_wp['name']} (Dist: {dist_to_thr:.1f} NM, Alt: {target_wp.get('alt', 0):.0f} FT)")
                            else:
                                # Na vetoração (fases 1 a 5), o alvo deve ser sempre o INÍCIO da aproximação (GS01, index 0)
                                if idx != 0:
                                    nav_state["active_waypoint_index"] = 0
                                    target_wp = wps[0]
                                    nav_state["lat_orig"] = target_wp["lat"]
                                    nav_state["lon_orig"] = target_wp["lon"]
                                    if not nav_state.get("vnav_custom_set"):
                                        nav_state["vnav_target_alt"] = target_wp.get("alt")
                                        vnav_alt_text = str(int(target_wp.get("alt", 0))) if target_wp.get("alt") else ""
                                    print("GS Sintético: Vetoração ativa. Alvo fixado no início da aproximação (GS01).")
                                    
                            # Se chegou na cabeceira/pista após aproximação/final (fase 6/7):
                            long_pos = nav_state.get("longitudinal_from_origin")
                            in_runway_zone = (dist_to_thr < 0.30) or (long_pos is not None and -0.20 <= long_pos <= 0.85 and abs(nav_state.get("xtk", 0.0)) < 0.25)
                            # Pouso ou toque-e-arremete exige alinhamento com o sentido da pista (inbound, margem segura de ±40°):
                            inbound_aligned = (hdg_diff_inbound <= 40.0)
                            if in_runway_zone and idx == len(wps) - 1 and v_phase >= 6 and not nav_state.get("gs_takeoff_active"):
                                if data.get("speed", 0.0) < 35.0 and inbound_aligned:
                                    nav_state["navigation_mode"] = "route_finished"
                                    nav_state["active"] = False
                                    print("Chegada à cabeceira atingida. Pouso finalizado e parada na pista.")
                                elif long_pos is not None and long_pos > 0.35 and data.get("speed", 0.0) >= 45.0 and inbound_aligned:
                                    # Só dispara toque e arremete se ULTRAPASSAR a cabeceira (> +0.35 NM pista adentro) mantendo velocidade de voo (>= 45 kt)!
                                    print("*** ARREMETIDA / TOQUE E ARREMETE DETECTADO APÓS A CABECEIRA! Reiniciando rampa de 50FT ***")
                                    nav_state["circuit_side"] = None
                                    nav_state["circuit_side_locked"] = False
                                    nav_state["base_turn_started"] = False
                                    nav_state["circuit_reset_pending"] = True
                                    nav_state["vectoring_phase"] = 1
                                    nav_state["gs_takeoff_active"] = True
                                    nav_state["active_waypoint_index"] = 0
                                    nav_state["abeam_passed"] = False
                                    nav_state["navigation_mode"] = "route"
                                    nav_state["active"] = True
                            
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
                        # Durante a vetoração (perna paralela/base de um retorno a waypoints manuais),
                        # passar pelo través ou começar a se afastar do WP alvo NÃO é "waypoint atingido":
                        # o piloto está montando o circuito. Só avança ao capturar de fato (distância)
                        # ou depois, já interceptando/na final (fases 6 e 7).
                        in_vectoring_leg = (not nav_state.get("active_procedure")) and (1 <= nav_state.get("vectoring_phase", 0) <= 5)
                        active_procedure = nav_state.get("active_procedure")
                        max_abeam_cpa_dist = 3.0 if active_procedure else 2.5
                        passed_abeam = abs(bearing_diff) >= 95 and dist_to_wp < max_abeam_cpa_dist and not in_vectoring_leg and (active_procedure or abs(nav_state.get("xtk", 0.0)) <= 0.45)
                    
                        # Lógica CPA (Closest Point of Approach)
                        prev_dist = nav_state.get("prev_dist_to_wp")
                        passed_cpa = False
                        if prev_dist is not None and not in_vectoring_leg:
                            if active_procedure:
                                if dist_to_wp > prev_dist + 0.05 and dist_to_wp < 3.0:
                                    passed_cpa = True
                            else:
                                # Em waypoints manuais, CPA só confirma passagem muito próximo ao fixo (<= 0.12 NM)
                                if dist_to_wp > prev_dist + 0.02 and dist_to_wp <= 0.12 and abs(bearing_diff) >= 75:
                                    passed_cpa = True
                        nav_state["prev_dist_to_wp"] = dist_to_wp
                    
                        # Antecipação de curva (Fly-By) dinâmico
                        turn_anticipation = False
                        if active_procedure and not is_mapt:
                            # Identifica o próximo fixo (faz looping para o IAF se estiver no MAHF)
                            next_idx_temp = 0 if idx == len(wps) - 1 else idx + 1
                            next_wp_temp = wps[next_idx_temp]
                        
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
                    
                        # Avaliação de Captura Normal vs Overshoot
                        should_advance = False
                        is_overshoot = False
                        reason = ""

                        if active_procedure:
                            # Procedimentos formais (Cartas IFR SBSJ): mantém lógica existente de fly-by e captura
                            if dist_to_wp < WP_CAPTURE_RADIUS_NM or passed_abeam or turn_anticipation or passed_cpa:
                                should_advance = True
                                if dist_to_wp < WP_CAPTURE_RADIUS_NM: reason = "distância"
                                elif turn_anticipation: reason = "antecipação de curva (fly-by)"
                                elif passed_abeam: reason = "passagem pelo través"
                                else: reason = "afastamento (CPA)"
                        else:
                            # Waypoints Manuais: Captura precisa de 0.05 NM (~90m) ou través/CPA <= 0.08 NM (~150m)
                            now_wp = time.time()
                            last_adv = nav_state.get("last_wp_advance_time", 0)
                            time_since_adv = now_wp - last_adv
                            
                            # Proteção anti-runaway para waypoints manuais:
                            can_advance = (time_since_adv >= 2.0) and (not in_vectoring_leg)
                            MANUAL_CAPTURE_NM = 0.05

                            if dist_to_wp < MANUAL_CAPTURE_NM and can_advance:
                                should_advance = True
                                reason = f"sobrevoo ({dist_to_wp*1852:.0f}m)"
                            elif (passed_abeam or passed_cpa) and can_advance:
                                if dist_to_wp <= 0.08:
                                    should_advance = True
                                    reason = f"través/CPA ({dist_to_wp*1852:.0f}m)"
                                else:
                                    # Passou afastado e o fixo ficou para trás (> 0.08 NM):
                                    should_advance = True
                                    is_overshoot = True
                                    reason = f"OVERSHOOT (dist={dist_to_wp:.2f} NM)"

                        if should_advance:
                            nav_state["last_wp_advance_time"] = time.time()
                            nav_state["circuit_side"] = None
                            nav_state["circuit_side_locked"] = False
                            nav_state["downwind_established"] = False
                            nav_state["prev_dist_to_wp"] = None
                            nav_state["vectoring_phase"] = 0
                            next_idx = idx + 1

                            if is_overshoot and next_idx < len(wps):
                                next_wp = wps[next_idx]
                                nav_state["overshoot_banner"] = f"IA: OVERSHOOT EM {target_wp['name']} ({dist_to_wp:.1f} NM). Sequenciando rota: avançando para {next_wp['name']}."
                                nav_state["overshoot_from"] = target_wp['name']
                                nav_state["overshoot_to"] = next_wp['name']
                                nav_state["overshoot_time"] = time.time()
                                print(f"*** OVERSHOOT DETECTADO em {target_wp['name']} (Dist: {dist_to_wp:.2f} NM). Sequenciando para {next_wp['name']}. ***")
                            else:
                                print(f"Waypoint {target_wp['name']} atingido (por {reason}, dist={dist_to_wp:.3f} NM). Avançando para próximo.")
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
                                        if nav_state.get("gs_synthetic_active"):
                                            nav_state["lat_orig"] = nav_state.get("gs_thr_lat", nav_state["lat_orig"])
                                            nav_state["lon_orig"] = nav_state.get("gs_thr_lon", nav_state["lon_orig"])
                                            nav_state["course"] = nav_state.get("gs_inbound_course", nav_state["course"])
                                            nav_state["course_true"] = nav_state.get("gs_inbound_course_true", nav_state["course_true"])
                                            bearing_mag = nav_state["course"]
                                            course_text = f"{int(bearing_mag):03}"
                                        else:
                                            # Waypoints Manuais: Calcula o curso geodésico exato conectando target_wp -> next_wp!
                                            nav_state["lat_orig"] = target_wp["lat"]
                                            nav_state["lon_orig"] = target_wp["lon"]
                                            leg_brg_true = NavMath.bearing_degrees(target_wp["lat"], target_wp["lon"], next_wp["lat"], next_wp["lon"])
                                            mag_var = data.get("mag_var", 0.0)
                                            leg_brg_mag = (leg_brg_true - mag_var + 360) % 360
                                            nav_state["course_true"] = leg_brg_true
                                            nav_state["course"] = leg_brg_mag
                                            bearing_mag = leg_brg_mag
                                            course_text = f"{int(leg_brg_mag):03}"
                                
                                    if not nav_state.get("vnav_custom_set"):
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
                                    nav_state["dist_to_wp"] = None
                                    vnav_alt_text = ""
                                    nav_state["overshoot_from"] = None
                                    nav_state["overshoot_to"] = None
                                    nav_state["overshoot_banner"] = "ROTA CONCLUIDA! Clique em RETORNAR WP01 para aproximar."
                                    nav_state["overshoot_time"] = time.time()
            else:
                nav_state["dist_to_wp"] = None

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
                    # Ativa o GS (barra VNAV) sempre que na interceptação/final (fase >= 6) ou dentro de 15 NM
                    if (dist_to_thr <= 15.0 or v_phase_current >= 6) and dist_to_thr > 0:
                        gs_alt = (rwy_elev + 50) + (dist_to_thr * 318.4)
                        nav_state["vnav_gs_alt"] = gs_alt

            # Alimenta a telemetria ao Co-Piloto Neural de Reinterceptação (sem travar o Pygame)
            dist_param_ai = nav_state.get("gs_dist_to_threshold") if nav_state.get("gs_synthetic_active") else nav_state.get("dist_to_wp")
            ai_copilot.update_telemetry(
                runway_course=nav_state.get("course", 0.0),
                current_heading=data.get("heading", 0.0),
                xtk_nm=xtk_nm,
                dist_nm=dist_param_ai,
                ias_kt=data.get("speed", 100.0),
                v_phase=v_phase_current,
                is_active=nav_state.get("active", False)
            )
            nav_state["ai_online"] = ai_copilot.is_online()

            if v_phase_current == 0 or v_phase_current == 7:
                # Se a IA estiver ativa, ela é a autoridade única e exclusiva (zero conflito com algoritmo)
                if ai_copilot.is_online():
                    ai_hdg, ai_res = ai_copilot.get_guidance()
                    nav_state["target_heading"] = ai_hdg
                    nav_state["target_heading_float"] = ai_copilot.get_smooth_heading()
                    if ai_res and ai_res.get("reason"):
                        nav_state["ai_reason"] = ai_res.get("reason")
                else:
                    # Algoritmo matemático original (preservado como fallback de segurança se a IA cair)
                    max_ang = 15.0 if nav_state.get("active_procedure") else 45.0
                    dist_thr = nav_state.get("gs_dist_to_threshold")
                    
                    if nav_state.get("gs_synthetic_active") and dist_thr is not None and dist_thr < 2.0:
                        deadband_final = 0.005  # ~9 metros (largura da faixa central da pista)
                        sens_final = 80.0
                        max_ang_final = min(25.0, max_ang)
                        target_heading_true = NavMath.calculate_intercept_heading(
                            nav_state["course_true"], xtk_nm,
                            sensitivity=sens_final, max_intercept=max_ang_final, deadband_nm=deadband_final,
                            dist_to_wp_nm=dist_thr
                        )
                    else:
                        dist_wp_val = nav_state.get("gs_dist_to_threshold") if nav_state.get("gs_synthetic_active") else nav_state.get("dist_to_wp")
                        target_heading_true = NavMath.calculate_intercept_heading(
                            nav_state["course_true"], xtk_nm, max_intercept=max_ang, dist_to_wp_nm=dist_wp_val
                        )
                    mag_var = data.get("mag_var", 0.0)
                    fallback_hdg = int((target_heading_true - mag_var + 360) % 360)
                    nav_state["target_heading"] = fallback_hdg
                    nav_state["target_heading_float"] = float(fallback_hdg)
            
            elif v_phase_current == 6:
                # INTERCEPTAÇÃO
                if ai_copilot.is_online():
                    ai_hdg, ai_res = ai_copilot.get_guidance()
                    nav_state["target_heading"] = ai_hdg
                    nav_state["target_heading_float"] = ai_copilot.get_smooth_heading()
                    if ai_res and ai_res.get("reason"):
                        nav_state["ai_reason"] = ai_res.get("reason")
                else:
                    max_ang = 30.0
                    dist_wp_val = nav_state.get("gs_dist_to_threshold") if nav_state.get("gs_synthetic_active") else nav_state.get("dist_to_wp")
                    target_heading_true = NavMath.calculate_intercept_heading(
                        nav_state["course_true"], xtk_nm, max_intercept=max_ang, dist_to_wp_nm=dist_wp_val
                    )
                    mag_var = data.get("mag_var", 0.0)
                    fallback_hdg = int((target_heading_true - mag_var + 360) % 360)
                    nav_state["target_heading"] = fallback_hdg
                    nav_state["target_heading_float"] = float(fallback_hdg)
            
            elif 1 <= v_phase_current <= 5:
                # ── CIRCUITO (vetoração): fases 1 a 5 ────────────────────────────────────────
                # A referência é o CURSO DO WAYPOINT ALVO (curso gravado do WP1 / eixo da pista no 50FT),
                # e não o direct-to calculado no clique. A perna paralela é sempre a proa INVERSA desse curso.
                _wps_ref = nav_state.get("recorded_waypoints", [])
                _idx_ref = nav_state.get("active_waypoint_index", 0)
                if nav_state.get("navigation_mode") == "route" and 0 <= _idx_ref < len(_wps_ref):
                    pat_course_true = _wps_ref[_idx_ref].get("course_true", nav_state["course_true"])
                    pat_course_mag = _wps_ref[_idx_ref].get("course", nav_state["course"])
                else:
                    pat_course_true = nav_state.get("course_true", nav_state["course"])
                    pat_course_mag = nav_state["course"]
                nav_state["pattern_course"] = pat_course_mag  # a tela usa para "MANTER PERNA PARALELA PROA X"
                mag_var = data.get("mag_var", 0.0)
                c_side = nav_state.get("circuit_side")

                # Rampa CARREGADA com lado de circuito salvo: guia o avião para uma perna paralela
                # (PATTERN_OFFSET_NM do eixo) do MESMO lado da aproximação original, mesmo que ele
                # esteja do lado oposto da pista. O corte é suave (máx. 30°) para não virar corda bamba.
                PATTERN_OFFSET_NM = 2.0
                pattern_err = None
                long_val = nav_state.get("longitudinal_from_origin")
                # pattern_err só atua após cruzar o través (long_val <= 0.3 NM), na perna do vento.
                # Antes do través (long_val > 0.3 NM), o avião voa na proa paralela pura (ex: 082°)
                # permitindo que o piloto aproxime e cruze a pista livremente sem ser forçado lateralmente.
                if (nav_state.get("circuit_side_locked") and c_side and nav_state.get("gs_synthetic_active")
                        and not nav_state.get("active_procedure") and nav_state.get("gs_thr_lat") is not None
                        and (long_val is None or long_val <= 0.3)):
                    xtk_axis_leg, _ = NavMath.calculate_xtk_by_course(
                        data["lat"], data["lon"],
                        nav_state["gs_thr_lat"], nav_state["gs_thr_lon"],
                        nav_state["gs_inbound_course_true"]
                    )
                    desired_off = PATTERN_OFFSET_NM if c_side == "R" else -PATTERN_OFFSET_NM
                    pattern_err = xtk_axis_leg - desired_off  # + = avião à direita da linha desejada (olhando no sentido do pouso)

                # Direct-To para o aeródromo (Retorno à distância > 4.0 NM no 50FT sem circuito escolhido):
                is_direct_to = False
                dist_thr_direct = nav_state.get("gs_dist_to_threshold")
                if (nav_state.get("gs_synthetic_active") and not nav_state.get("active_procedure")
                        and nav_state.get("is_return_approach", False)
                        and not nav_state.get("gs_takeoff_active", False)
                        and c_side is None and nav_state.get("gs_thr_lat") is not None
                        and dist_thr_direct is not None and dist_thr_direct > 4.0):
                    thr_b_true = NavMath.bearing_degrees(data["lat"], data["lon"], nav_state["gs_thr_lat"], nav_state["gs_thr_lon"])
                    direct_hdg = int((thr_b_true - mag_var + 360) % 360)
                    nav_state["direct_to_airport"] = True
                    nav_state["direct_to_heading"] = direct_hdg
                    nav_state["direct_to_dist"] = dist_thr_direct
                    is_direct_to = True
                else:
                    nav_state["direct_to_airport"] = False

                if v_phase_current == 5:
                    # PERNA BASE: exatos 90 graus em relação ao curso, sem correção de XTK!
                    if c_side == "L":
                        base_true = (pat_course_true + 90) % 360
                    else:
                        base_true = (pat_course_true - 90 + 360) % 360
                    nav_state["target_heading"] = int((base_true - mag_var + 360) % 360)
                elif is_direct_to:
                    # Voando direto para a cabeceira / aeródromo enquanto > 4 NM e sem circuito escolhido
                    nav_state["target_heading"] = direct_hdg
                elif v_phase_current in (2, 3, 4) or c_side:
                    # PERNA PARALELA / AFASTAMENTO: proa recíproca ao curso
                    reciprocal_true = (pat_course_true + 180) % 360
                    if pattern_err is not None:
                        # Voando na recíproca, esquerda/direita em relação à linha se invertem (-pattern_err)
                        reciprocal_true = NavMath.calculate_intercept_heading(
                            reciprocal_true, -pattern_err, sensitivity=20.0, max_intercept=30.0
                        )
                    nav_state["target_heading"] = int((reciprocal_true - mag_var + 360) % 360)
                else:
                    # Fase 1 ainda sem lado definido:
                    if nav_state.get("is_return_approach", False):
                        # Em aproximação de retorno a <= 4 NM aguardando seleção, sugere proa paralela
                        reciprocal_true = (pat_course_true + 180) % 360
                        nav_state["target_heading"] = int((reciprocal_true - mag_var + 360) % 360)
                    else:
                        # Decolagem / subida inicial: mantém o curso da pista
                        nav_state["target_heading"] = int(pat_course_mag)
            
            else:
                nav_state["target_heading"] = int(nav_state["course"])
        else:
            nav_state["xtk"] = 0.0
            nav_state["target_heading"] = 0

        # --- MODO LNAV / MONITORAMENTO DE CURSO EM TEMPO REAL ---
        nav_state["log_final_active"] = False
        v_phase_current = nav_state.get("vectoring_phase", 0)
        
        # O monitoramento contínuo de curso atua na FINAL (fase 7) ou perna de rota (fase 0),
        # NUNCA durante as fases de circuito/vetoração (1 a 6).
        # - Para Waypoints Manuais e 50FT: monitora o curso SEMPRE, independente do botão LOG!
        # - Para Cartas IFR formais: monitora o curso somente se o botão LOG estiver ativado.
        should_track_course = False
        if nav_state["active"] and data.get("connected") and nav_state.get("navigation_mode") != "route_finished" \
                and v_phase_current not in (1, 2, 3, 4, 5, 6):
            if nav_state.get("active_procedure"):
                should_track_course = bool(nav_state.get("log_mode"))
            else:
                should_track_course = True
                
        if should_track_course:
            log_track = data.get("track", data.get("heading", 0.0))
            log_dev = NavMath.normalize_angle(log_track - nav_state["course"])
            
            # Só ativa se o avião estiver aproximado/alinhado com o eixo final
            is_active_visually = False
            if nav_state.get("gs_synthetic_active"):
                # No 50 pés, só arma a UI de interceptação quando estabelecido na final
                if abs(log_dev) <= 45.0 and abs(nav_state.get("xtk", 0.0)) <= 1.5:
                    is_active_visually = True
            else:
                # Nos outros modos (IFR/Waypoints), arma quando ingressar na final ou alinhado com a perna (<= 45°)
                if abs(log_dev) <= 45.0:
                    is_active_visually = True
                
            if is_active_visually:
                dist_param = nav_state.get("gs_dist_to_threshold") if nav_state.get("gs_synthetic_active") else None
                dist_wp_val = nav_state.get("dist_to_wp")
                nav_state["log_final_active"] = True
                nav_state["log_current_course"] = log_track
                
                # Se a IA estiver ativa, usa exclusivamente os comandos da IA (zero conflito)
                if ai_copilot.is_online():
                    ai_target = nav_state.get("target_heading")
                    if ai_target is not None:
                        nav_state["log_target_course"] = int(round(ai_target)) % 360
                        
                        # DIREÇÃO DE CURVA: Calculada SEMPRE em relação à proa atual do avião!
                        # (Evita o bug de mandar curvar à direita quando o avião precisa curvar à esquerda)
                        cur_hdg = data.get("heading", log_track)
                        diff_to_target = (ai_target - cur_hdg + 180.0) % 360.0 - 180.0
                        
                        dist_thr = nav_state.get("gs_dist_to_threshold")
                        xtk_val = abs(nav_state.get("xtk", 0.0))
                        
                        # 1. ESTABILIZAÇÃO SAGRADA NO EIXO E NA FINAL CURTA:
                        # Se estiver alinhado no eixo da pista (< 0.020 NM ~ 37m) ou na final curta (< 1.5 NM e < 0.035 NM):
                        # Fixa "MANTER CURSO" em verde sólido absoluto! Zero ordens espúrias de curva!
                        is_on_centerline = (xtk_val <= 0.020)
                        is_final_aligned = (dist_thr is not None and dist_thr < 1.5 and xtk_val <= 0.035)
                        
                        if is_on_centerline or is_final_aligned:
                            nav_state["log_turn"] = None
                            nav_state["log_correcting"] = False
                            nav_state["log_intercepting"] = False
                            nav_state["log_target_course"] = int(round(nav_state.get("course", ai_target))) % 360
                        else:
                            # 2. HISTERESE FIRME E FLUIDA (Elimina a gangorra de alternância rápida de telas):
                            # Dispara aviso de curva ("CURVA ESQ/DIR") quando a proa desviar por >= 5.0°!
                            # Para entrar em "MANTER XXX° INTERCEPTANDO", precisa alinhar a <= 2.5° da proa alvo.
                            # Enquanto o piloto mantiver a proa com pequenas correções (±2.5°), fica firme em MANTER INTERCEPTANDO!
                            was_corr = nav_state.get("log_correcting", False)
                            if abs(diff_to_target) >= 5.0:
                                nav_state["log_turn"] = "R" if diff_to_target > 0 else "L"
                                nav_state["log_correcting"] = True
                                nav_state["log_intercepting"] = True
                            elif abs(diff_to_target) <= 2.5:
                                nav_state["log_turn"] = None
                                nav_state["log_correcting"] = False
                                nav_state["log_intercepting"] = True
                            else:
                                if not was_corr:
                                    nav_state["log_turn"] = None
                                    nav_state["log_correcting"] = False
                                    nav_state["log_intercepting"] = True
                else:
                    # MODO FALLBACK: algoritmo matemático legado (desativado quando IA está ativa)
                    log_advice = NavMath.calculate_course_deviation_advice(
                        nav_state["course"], log_track,
                        xtk_nm=nav_state.get("xtk", 0.0),
                        was_correcting=nav_state.get("log_correcting", False),
                        dist_to_runway_nm=dist_param,
                        dist_to_wp_nm=dist_wp_val
                    )
                    nav_state["log_deviation"] = log_advice["deviation"]
                    nav_state["log_correcting"] = log_advice["correcting"]
                    nav_state["log_intercepting"] = log_advice["intercepting"]
                    nav_state["log_turn"] = log_advice["turn"]
                    nav_state["log_target_course"] = log_advice["target_course"]
                
        if not nav_state["log_final_active"]:
            nav_state["log_correcting"] = False

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

        # ── LOG CSV Lógica ──
        # Regra Universal de Voo Real: apertou o botão LOG -> grava imediatamente no LOG/REAL!
        should_log_now = bool(nav_state.get("log_mode", False))

        if should_log_now:
            if not nav_state.get("csv_logging_active"):
                sub_folder = "PC" if os.name == "nt" else "REAL"
                log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "LOG", sub_folder)
                os.makedirs(log_dir, exist_ok=True)
                nav_state["csv_log_filename"] = get_next_log_filename(log_dir)
                nav_state["csv_logging_active"] = True
                nav_state["csv_last_time"] = time.time()
                try:
                    with open(nav_state["csv_log_filename"], "w", encoding="utf-8") as f:
                        f.write("Time;Lat;Lon;Alt;Speed;Heading;Course;Pitch;Bank;AoA\n")
                        f.flush()
                    print(f"[LOG REAL] Iniciou gravação de LOG CSV em: {nav_state['csv_log_filename']}")
                except Exception as e:
                    print(f"[LOG ERRO] Erro ao criar log CSV: {e}")
                    nav_state["csv_logging_active"] = False
            else:
                now = time.time()
                if now - nav_state["csv_last_time"] >= 1.0:
                    nav_state["csv_last_time"] = now
                    try:
                        import datetime
                        time_str = datetime.datetime.fromtimestamp(now).strftime('%H:%M:%S')
                        
                        # Formata os números substituindo o ponto decimal por vírgula para o Excel PT-BR
                        lat = f"{data.get('lat',0):.6f}".replace('.', ',')
                        lon = f"{data.get('lon',0):.6f}".replace('.', ',')
                        alt = f"{data.get('alt',0):.1f}".replace('.', ',')
                        spd = f"{data.get('speed',0):.1f}".replace('.', ',')
                        hdg = f"{data.get('heading',0):.1f}".replace('.', ',')
                        crs = f"{nav_state.get('course',0):.1f}".replace('.', ',')
                        pit = f"{data.get('pitch',0):.2f}".replace('.', ',')
                        bnk = f"{data.get('bank',0):.2f}".replace('.', ',')
                        aoa = f"{data.get('aoa',0):.2f}".replace('.', ',')
                        
                        with open(nav_state["csv_log_filename"], "a", encoding="utf-8") as f:
                            row = f"{time_str};{lat};{lon};{alt};{spd};{hdg};{crs};{pit};{bnk};{aoa}\n"
                            f.write(row)
                            f.flush()
                    except Exception as e:
                        print(f"[LOG ERRO] Falha ao gravar linha: {e}")
        else:
            if nav_state.get("csv_logging_active"):
                print(f"[LOG REAL] Finalizou gravação de LOG CSV: {nav_state['csv_log_filename']}")
                nav_state["csv_logging_active"] = False
                nav_state["csv_log_filename"] = None

        # Telemetria ao vivo em disco para monitoramento e diagnóstico em tempo real (a cada 0.5s)
        now_live = time.time()
        if "_last_live_dump" not in nav_state or now_live - nav_state["_last_live_dump"] >= 0.5:
            nav_state["_last_live_dump"] = now_live
            try:
                live_snapshot = {
                    "time": now_live,
                    "connected": bool(data.get("connected")),
                    "alt": round(float(data.get("alt", 0)), 1),
                    "speed": round(float(data.get("speed", 0)), 1),
                    "heading": round(float(data.get("heading", 0)), 1),
                    "track": round(float(data.get("track", data.get("heading", 0.0))), 1),
                    "satellites": int(data.get("satellites", 0)),
                    "bank": round(float(data.get("bank", 0)), 1),
                    "pitch": round(float(data.get("pitch", 0)), 1),
                    "course": round(float(nav_state.get("course", 0.0) or 0.0), 1),
                    "xtk_nm": round(float(nav_state.get("xtk", 0.0) or 0.0), 4),
                    "target_heading": nav_state.get("target_heading"),
                    "target_heading_float": round(float(nav_state.get("target_heading_float", 0.0) or 0.0), 2),
                    "turn_rate_dps": round(float(nav_state.get("turn_rate_dps", 0.0) or 0.0), 2),
                    "log_turn": nav_state.get("log_turn"),
                    "log_target_course": nav_state.get("log_target_course"),
                    "log_correcting": nav_state.get("log_correcting"),
                    "log_intercepting": nav_state.get("log_intercepting"),
                    "ai_online": nav_state.get("ai_online"),
                    "ai_reason": nav_state.get("ai_reason"),
                    "v_phase": nav_state.get("vectoring_phase", 0),
                    "gs_dist": nav_state.get("gs_dist_to_threshold"),
                }
                with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "live_telemetry.json"), "w", encoding="utf-8") as f_live:
                    json.dump(live_snapshot, f_live, ensure_ascii=False)
            except Exception:
                pass

        # RENDERIZAÇÃO
        canvas.fill(BLACK)

        if data.get("connected"):
            # Desenha o Horizonte Artificial na Esquerda (Pitch e Bank com suavização ultra fluida)
            smooth_pitch = nav_state.get("_smooth_pitch", data.get("pitch", 0.0))
            smooth_bank = nav_state.get("_smooth_bank", data.get("bank", 0.0))
            gui.draw_horizon(canvas, smooth_pitch, smooth_bank)

            # Desenha as barras do Flight Director sobre o horizonte (se ativo e configurado)
            gui.draw_flight_director(canvas, nav_state, nav_state.get("_smooth_alt", data.get("alt", 0)))

            # Desenha a miniatura do avião por cima do Flight Director
            gui.draw_aircraft_reference(canvas)

            # Desenha a escala vertical de waypoints no canto esquerdo
            gui.draw_waypoint_scale(
                canvas,
                nav_state["recorded_waypoints"],
                nav_state["active_waypoint_index"],
                nav_state["navigation_mode"]
            )

            # Desenha a marca oficial NAVEG-AI no canto inferior esquerdo (embaixo do horizonte)
            gui.draw_brand_logo(canvas)

            # Desenha as Caixas de Telemetria Básica com filtragem de movimento contínuo
            # HDG (Proa Magnética Suavizada)
            hdg_border = BLUE if active_input == "cal_hdg" else GRAY
            smooth_hdg = int(round(nav_state.get("_smooth_hdg", data.get("heading", 0.0)))) % 360
            gui.draw_data_box(canvas, HDG_RECT, "HDG", f"{smooth_hdg:03}°", border_color=hdg_border)
            # TRK (Rumo Real Suavizado e Travado na Proa em Baixa Velocidade)
            smooth_trk = int(round(nav_state.get("_smooth_trk", data.get("track", data.get("heading", 0.0))))) % 360
            # TRK ocultado na versao TESTADA para painel limpo
            # gui.draw_data_box(canvas, TRK_RECT, "TRK", f"{smooth_trk:03}°", color=YELLOW)
            # SPD / GS (Velocidade de Solo no modo Real Raspberry Pi, IAS no simulador)
            spd_label = "GS" if mode_chosen == "pi" else "SPD"
            smooth_spd = int(round(nav_state.get("_smooth_speed", data.get("speed", 0.0))))
            gui.draw_data_box(canvas, SPD_RECT, spd_label, f"{smooth_spd}")
            # ALT (Altitude Suavizada)
            smooth_alt = int(round(nav_state.get("_smooth_alt", data.get("alt", 0.0))))
            gui.draw_data_box(canvas, ALT_RECT, "ALT", f"{smooth_alt}")

            # Desenha o Painel de Navegação Lateral na Direita (com suporte a foco duplo de input)
            try:
                gui.draw_navigation_panel(canvas, data, nav_state, mouse_pos, active_input, course_text, tolerance_text, gs_rwy_alt_text)
            except Exception as _ui_err:
                print(f"[UI WARN] draw_navigation_panel: {_ui_err}", flush=True)
            
            # Desenha o Painel de Navegação Vertical na Direita Extra
            try:
                gui.draw_vnav_panel(canvas, data, nav_state, mouse_pos, active_input, vnav_alt_text, vnav_tol_text, vnav_cal_text)
            except Exception as _ui_err:
                print(f"[UI WARN] draw_vnav_panel: {_ui_err}", flush=True)
            # Desenha o Painel de Procedimento (Cartas) na Direita Extra Inferior
            gui.dropdown_rects, gui.dir_dropdown_rects = gui.draw_procedure_panel(
                canvas, mouse_pos, selected_chart_name, available_chart_keys, show_chart_dropdown, nav_state, show_dir_dropdown
            )

            # Desenha o Teclado Touchscreen se estiver ativo
            gui.draw_touch_keypad(canvas, mouse_pos)
            # Desenha o Painel de Nivelamento de Atitude se ativo
            gui.draw_attitude_panel(canvas, mouse_pos, data.get("pitch", 0.0), data.get("bank", 0.0), getattr(source, 'calibration', {}))
        else:
            # Sem conexão com simulador/sensores
            if data.get("waiting_for_flight"):
                gui.draw_text(canvas, "Conectado. Aguardando voo carregar...", (1340 // 2, 900 // 2), gui.f_title, YELLOW, align="center")
            else:
                gui.draw_text(canvas, "Sem conexao com o simulador / sensores", (1340 // 2, 900 // 2), gui.f_title, (255, 50, 50), align="center")

        # Escala e desenha na janela física
        draw_scaled_canvas(screen, canvas, window_size)
        pygame.display.flip()
        # Crava a taxa de execucao em exatamente 30 frames por segundo (evita sobrecarga de CPU)
        clock.tick(30)

    # Limpeza final e reset do offset temporário de proa da pista ao encerrar (pedido do Comandante)
    try:
        cal_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sensor_calibration.json")
        if os.path.exists(cal_path):
            with open(cal_path, "r", encoding="utf-8") as f:
                c_data = json.load(f)
            if c_data.get("heading_offset", 0.0) != 0.0:
                c_data["heading_offset"] = 0.0
                with open(cal_path, "w", encoding="utf-8") as f:
                    json.dump(c_data, f, indent=4)
                print("[OK] Offset de alinhamento de proa da pista resetado para 0.0 ao fechar.")
    except Exception as e:
        print(f"Erro ao resetar offset de proa ao fechar: {e}")

    ai_copilot.stop()
    source.disconnect()
    # Encerra os subsistemas graficos do Pygame e libera a memoria de video
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
