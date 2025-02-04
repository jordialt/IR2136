import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/usuario/Documentos/GitHub/IR2136/prac3_ws/install/my_drone_package'
