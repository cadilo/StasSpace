import numpy as np 
from port import Port
from chain import inverse_kinematic, forward_kinematic
import argparse
import json
import time

# Данные для калибровки
# для J0 -
# для J1 - 1 градус = 300 шагов
# для J2 - 1 градус = 285 шагам

#usage_port = '/dev/pts/8'
#usage_port = '/dev/ttyUSB0'
usage_port = None

class RobotCalibration():
    k = np.array([
        90 /     152.0,
        44 /     -70.0,
        90 /    -182.5,
        90 /     113.5,
        90 /  115550.0,
        90 /      90.0,
    ])

    def units_to_degrees(self, units=(0, 0, 0, 0, 0, 0)):
        return [ round(x,4) for x in np.array(units) * self.k ]

    def degrees_to_units(self, degrees=(0, 0, 0, 0, 0, 0)):
        return [ round(x,4) for x in np.array(degrees) / self.k ]

class RobotState():
    def __init__(self, joints=(0, 0, 0, 0, 0, 0), v=0, operation='', error=None):
        self.joints = joints
        self.v = v
        self.operation = operation
        self.error = error

        self.mins = (-90, -50, -60, -90, -90, -180)
        self.maxs = ( 90, 50, 60, 90, 90, 180)
    
class Robot():
    def __init__(self, port=usage_port, timeout=10000):
        self.port = Port(port)
        self.calibration = RobotCalibration()
        self.state = RobotState()
        self.timeout = timeout

    def set_speed(self, v):
        try:
            assert 1 <= v <= 100, f'Speed not in 1 <= {v} <= 100!'
            self.port.set_speed(v)
            self.state.v = v
            self.state.operation = f'set_speed(v={v})'
            self.state.error = None
        except AssertionError as e:
            self.state.error = str(e)
            raise e
        
    def get_speed(self):
        return self.state.v

    def set_joint_pos(self, joints=(0, 0, 0, 0, 0, 0)):
        try:
            # for j_min, j, j_max in zip(self.state.mins, joints, self.state.maxs):
            #     assert j_min <= j <= j_max, f'Joint not in {j_min} <= {j} <= {j_max}!'
            #joints_u = self.calibration.degrees_to_units(joints)
            j1, j2, j3, j4, j5, j6 = joints
            self.port.G00(j1, j2, j3, j4, j5, j6)
            #self.port.is_ready(self.timeout)
            self.state.joints = joints
            self.state.operation = f'set_joint_pos(joints={joints})'
            self.state.error = None
        except AssertionError as e:
            self.state.error = str(e)
            raise e

    def get_joint_pos(self):
        return self.state.joints

    def is_ready(self):
        return self.state.error is None

    def start_programm(self, filename):
        print("Вызван метод start_programm")

        try:
            # Читаем позиции из JSON файла
            with open(filename, 'r') as json_file:
                positions = json.load(json_file)

            for position in positions:
                # Формируем команду G00
                self.port.G00(position[0][1], position[1][1], position[2][1], position[3][1], position[4][1], position[5][1])
                time.sleep(2)
                # if self.port.G01() == "position complete":
                #     continue
                # else:
                #     print("Позиция не достигнута")

        finally:
            self.port.G00(0, 0, 0, 0, 0, 0)
            #print('Последовательный порт закрыт.')

    def write_programm(self, filename):
        print("Вызван метод write_programm")
        positions = []  # Объявляем positions как список
        try:
            while True:
                command = input("Введите write для записи положения (или 'exit' для завершения): ")
                if command.lower() == 'exit':
                    break
                
                # Отправляем команду
                response = self.port.G01()
                # Получаем ответ от устройства
                print(f'Ответ получен: {response}')
                
                response_data = []
                for pair in response.split():
                    if ':' in pair:  # Проверяем наличие ':' в строке
                        key, value = pair.split(':')
                        key = key.strip()  # Убираем лишние пробелы
                        value = value.strip()  # Убираем лишние пробелы
                        if value:  # Проверяем, что value не пустое
                            if value.isdigit():  # Проверка, что value является числом
                                response_data.append((key, int(value)))  
                            else:
                                print(f"Неправильное значение: {value}")  # Предупреждение, если значение не целое число
                        else:
                            print(f"Пустое значение для ключа: {key}")  # Уведомление о пустом значении
                    else:
                        print(f"Неправильный формат ответа: {pair}")

                positions.append(response_data)  # Добавляем response_data в positions

        finally:
            # Сохраняем ответы в JSON файл
            with open(f'{filename}', 'w') as json_file:
                json.dump(positions, json_file, indent=4)  # Сохраняем в формате JSON
        
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    ARGS = [
        ('-X',  '--X',  float,   0, 'X position MM.'),
        ('-Y',  '--Y',  float, 430, 'Y position MM.'),
        ('-Z',  '--Z',  float, 277, 'Z position MM.'),
        ('-rX', '--rX', float,   0, 'X tool rotation, degrees.'),
        ('-rY', '--rY', float,   0, 'Y tool rotation, degrees.'),
        ('-rZ', '--rZ', float,   0, 'Z tool rotation, degrees.'),
        ('-rot','--rot',  str, 'Z', 'Tool rotation axis.'),
        ('-j0', '--j0', int,   0, 'Joint #0 position, degrees.'),
        ('-j1', '--j1', int,   0, 'Joint #1 position, degrees.'),
        ('-j2', '--j2', int,   0, 'Joint #2 position, degrees.'),
        ('-j3', '--j3', int,   0, 'Joint #3 position, degrees.'),
        ('-j4', '--j4', int,   0, 'Joint #4 position, degrees.'),
        ('-j5', '--j5', int,   0, 'Joint #5 position, degrees.'),
        ('-s', '--speed', int,  70, 'Speed, percent.'),
        ('-sp', '--start_programm', str, None, 'Start programm filename'),
        ('-wp', '--write_programm', str, None, 'Write programm filename')
    ]
    try:
        robot = Robot()
    except Exception:
        print('working in emulated mode')
        robot = Robot(port=None)
    for a_short, a_long, a_type, a_value, a_help in ARGS:
        parser.add_argument(a_short, a_long, type=a_type, default=a_value, help=a_help)
    
    args = parser.parse_args()
    print(args)
    j0 = args.j0
    j1 = args.j1
    j2 = args.j2
    j3 = args.j3
    j4 = args.j4
    j5 = args.j5
    X = args.X
    Y = args.Y
    Z = args.Z
    rX = args.rX
    rY = args.rY
    rZ = args.rZ
    rot = args.rot
    speed = args.speed
    start_filename = args.start_programm
    write_filename = args.write_programm

    vec = [X, Y, Z]
    rot = [rX, rY, rZ]
    print(rot)

    if start_filename is not None:
        robot.start_programm(start_filename)
    elif write_filename is not None:
        robot.write_programm(write_filename)
    else:
        if sum([j0, j1, j2, j3, j4, j5]):
            print('joints')
            fk = forward_kinematic(np.radians(j0), 
                                   np.radians(j1),
                                   np.radians(j2), 
                                   np.radians(j3), 
                                   np.radians(j4), 
                                   np.radians(j5))
            robot.set_joint_pos((j0, j1, j2, j3, j4, j5))
            #time.sleep(10)
            robot.set_joint_pos((0, 0, 0, 0, 0, 0))
            
        else:
            print('coordinates')
            ik = inverse_kinematic(vec, rot)
            j0, j1, j2, j3, j4, j5 = [ round(np.degrees(x),2) for x in ik ][1:7]
            print(j0, j1, j2, j3, j4, j5)
            robot.set_joint_pos((j0, j1, j2, j3, j4, j5))

