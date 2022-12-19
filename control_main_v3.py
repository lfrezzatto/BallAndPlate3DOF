# Algorithm based on <https://jckantor.github.io/CBE30338/04.01-Implementing_PID_Control_with_Python_Yield_Statement.html>. Accessed in 10/24/22.

# Aperte 'q' para finalizar a execução da captura.

import math
import cv2
import numpy as np
import time
from multiprocessing import Value
import threading
import serial.tools.list_ports

# Estabelecendo comunicação com o arduino

ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()
portsList=[]

for onePort in ports:
    portsList.append(str(onePort))
    print(str(onePort))

# Escolha da porta COM do arduino
val = 5

for x in range(0, len(portsList)):
    if portsList[x].startswith('COM' + str(val)):
        portVar = 'COM' + str(val)
        print(portVar)

serialInst.baudrate = 9600
serialInst.port = portVar
serialInst.open()


# Definindo parâmetros
ts = 3
x_pos = Value('f', 0)
y_pos = Value('f', 0)

saturacao = 5

sp_x = 0
sp_y = 0

_kp = -0.3
_ki = -0.05
_kd = -0.45

def gera_msg(angulos):
    delta = [int(angulos[0]*100), int(angulos[1]*100), int(angulos[2]*100)]
    msg = str(delta)
    msg = msg[1:-1]
    return msg

def posicao_atual():
    while True:
        print(x_pos.value, y_pos.value)

def nothing(x):
    pass

def deg2rad(theta):
    return theta*math.pi/180

def rad2deg(theta):
    return theta*180/math.pi

def satura_sinal(valor, lim_sup, lim_inf):
    if valor > lim_sup:
        valor = lim_sup
    elif valor < lim_inf:
        valor = lim_inf
    else:
        pass
    return valor

def PID(Kp, Ki, Kd, MV_bar=0):      # Algoritmo de controle
    # initialize stored data
    e_prev = 0
    t_prev = -ts
    I = 0
    
    # initial control
    MV = MV_bar
    
    while True:
        # yield MV, wait for new t, PV, SP
        t, PV, SP = yield MV
        
        # PID calculations
        e = SP - PV
        
        P = Kp*e
        I = I + Ki*e*(t - t_prev)
        D = Kd*(e - e_prev)/(t - t_prev)
        
        MV = MV_bar + P + I + D
        
        # update stored data for next iteration
        e_prev = e
        t_prev = t

def cinematica_inversa(alfa, beta):     # Calcula a inclinação dos servos a partir dos ângulos enviados pelo controlador
    
    #constantes da planta
    rj = 0.12
    Rm = 0.071
    D = 0.095
    altura_media = 0.09

    #conversão para radianos
    alfa = deg2rad(alfa)
    beta = deg2rad(beta)

    #posição angular de B_i e A_i
    thetai = [deg2rad(0), deg2rad(120), deg2rad(240)]
    thetaj = [deg2rad(0), deg2rad(120), deg2rad(240)]

    l1 = altura_media - rj*math.sin(alfa)
    l2 = (rj*math.sin(alfa))/2 - (math.sqrt(3)*rj*math.sin(beta))/2 + altura_media
    l3 = (rj*math.sin(alfa))/2 + (math.sqrt(3)*rj*math.sin(beta))/2 + altura_media

    # o uso das funções seno e arcseno causam um pequeno erro no cálculo do ângulo. 
    # isso pode ser visto executando linha de código abaixo no terminal:
    # rad2deg(math.asin(deg2rad(30)))
    # retorna -> 31.57396132963207
    
    L = [l1, l2, l3]
    print(L)

    zi = [0, 0, 0]
    zj = [L[0], L[1], L[2]]

    a = [2*Rm*(zj[0]-zi[0]), 2*Rm*(zj[1]-zi[1]), 2*Rm*(zj[2]-zi[2])]

    c = [math.pow(L[0],2)-math.pow(D,2)+math.pow(Rm,2),
    math.pow(L[1],2)-math.pow(D,2)+math.pow(Rm,2),
    math.pow(L[2],2)-math.pow(D,2)+math.pow(Rm,2)]

    # assumindo bi sempre igual a 0:

    delta = [rad2deg(math.asin(c[0]/a[0])), rad2deg(math.asin(c[1]/a[1])), rad2deg(math.asin(c[2]/a[2]))]

    return delta

def captura_imagem(x_pos, y_pos):   # Usa a câmera como sensor para fazer o tracking da posição da bolinha
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()

    h, w, _ = frame.shape

    cv2.namedWindow('Dimensoes da area de captura')
    
    #Usar um dos conjuntos para X, Y, Raio abaixo. O segundo serve para definir valores padrões de inicialização, e o primeiro inicia no meio do slider.

    #cv2.createTrackbar('X', 'Dimensoes da area de captura', int(w/2), w, nothing)
    #cv2.createTrackbar('Y', 'Dimensoes da area de captura', int(h/2), h, nothing)
    #cv2.createTrackbar('Raio', 'Dimensoes da area de captura', int(w/4), int(w/2), nothing)
    
    cv2.createTrackbar('X', 'Dimensoes da area de captura', 318, w, nothing)
    cv2.createTrackbar('Y', 'Dimensoes da area de captura', 232, h, nothing)
    cv2.createTrackbar('Raio', 'Dimensoes da area de captura', 162, int(w/2), nothing)

    cv2.namedWindow('Ajuste deteccao bola')
    cv2.createTrackbar('LH', 'Ajuste deteccao bola', 26, 255, nothing)
    cv2.createTrackbar('LS', 'Ajuste deteccao bola', 94, 255, nothing)
    cv2.createTrackbar('LV', 'Ajuste deteccao bola', 255, 255, nothing)
    cv2.createTrackbar('UH', 'Ajuste deteccao bola', 255, 255, nothing)
    cv2.createTrackbar('US', 'Ajuste deteccao bola', 255, 255, nothing)
    cv2.createTrackbar('UV', 'Ajuste deteccao bola', 255, 255, nothing)

    frame_w = w
    frame_h = h
    while(cap.isOpened()):
        ret, frame = cap.read()

        if ret == True:

            x = cv2.getTrackbarPos('X', 'Dimensoes da area de captura')
            y = cv2.getTrackbarPos('Y', 'Dimensoes da area de captura')
            r = cv2.getTrackbarPos('Raio', 'Dimensoes da area de captura')

            cv2.circle(frame, (x,y), r if r >= 10 else 10, (255,255,255), 2)
            cv2.line(frame, (x,y), (x+r, y), (255, 255, 255),  2)
            cv2.putText(frame, ' A1', (x+r, y), cv2.FONT_HERSHEY_SIMPLEX, 0.71, (255, 255, 255), 1, cv2.LINE_AA)

            mask1 = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), np.array([0,0,255]), np.array([0,0,255]))
            cv2.circle(mask1, (x,y), r if r >= 10 else 10, (255,255,255), -1)
    
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            l_h = cv2.getTrackbarPos('LH', 'Ajuste deteccao bola')
            l_s = cv2.getTrackbarPos('LS', 'Ajuste deteccao bola')
            l_v = cv2.getTrackbarPos('LV', 'Ajuste deteccao bola')

            u_h = cv2.getTrackbarPos('UH', 'Ajuste deteccao bola')
            u_s = cv2.getTrackbarPos('US', 'Ajuste deteccao bola')
            u_v = cv2.getTrackbarPos('UV', 'Ajuste deteccao bola')

            l_b = np.array([l_h, l_s, l_v])
            u_b = np.array([u_h, u_s, u_v])

            mask = cv2.inRange(hsv, l_b, u_b)
            mask = cv2.dilate(mask, None, iterations=3)

            mask2 = cv2.bitwise_and(mask1, mask)

            res = cv2.bitwise_and(frame, frame, mask=mask2)

            cv2.circle(frame, (x,y), 0, (0,0,255), 4)

            O_x = x
            O_y = y 

            contours, _ = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                (x, y, w, h) = cv2.boundingRect(contour)

                if cv2.contourArea(contour) < 300:
                    continue
                else:
                    cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
                    saida_x = (int(x+w/2)-O_x)
                    saida_y = (-int(y+h/2)+O_y)
                    posicao_str = 'X: ' + str((saida_x/r)*0.3)[:4] + ' / Y: ' + str((saida_y/r)*0.3)[:4]
                    cv2.putText(frame, posicao_str, (35, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
                    #print(posicao_str)
                    x_pos.value = (saida_x/r)*0.3            #    normalização dos valores para uma escala de 0 a 30 cm
                    y_pos.value = (saida_y/r)*0.3

                    x_pos.value = saida_x
                    y_pos.value = saida_y
                
            
            cv2.imshow('frame', frame)
            #cv2.imshow('mask', mask)
            #cv2.imshow('mask1', mask1)
            
            # DESCOMENTAR A LINHA ABAIXO (MASK2 PARA AJUSTAR DETECÇÃO DE COR. PADRÃO: LARANJA.)
            cv2.imshow('mask2', mask2)
            #cv2.imshow('res', res)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

    cap.release()
    cv2.destroyAllWindows()

def main_loop():

    while(True):
        t = time.time()
        pv_x = x_pos.value
        pv_y = y_pos.value
        
        mv_x = satura_sinal(controlador_x.send([t, pv_x, sp_x]), saturacao, -saturacao)
        mv_y = satura_sinal(controlador_y.send([t, pv_y, sp_y]), saturacao, -saturacao)

        delta = cinematica_inversa(mv_x, mv_y)

        #print(delta)

        # Enviando ângulos para o arduino
        command = gera_msg(delta)
        command = command + (' \n')
        #print(command)
        print("X: " +str(x_pos.value) + " \ Y: " + str(y_pos.value))
        serialInst.write(command.encode('utf-8'))


        time.sleep(ts)


if __name__ == '__main__':

    controlador_x = PID(_kp, _ki, _kd) # Definindo parâmetros do controlador
    controlador_y = PID(_kp, _ki, _kd)

    controlador_x.send(None) # Inicializando o controlador
    controlador_y.send(None)
    
    #threading.Timer(1, main_loop).start()
    #threading.Thread(target=captura_imagem(x_pos, y_pos)).start()
