# EvolutionBoard STM32F4XX from Aliexpress BLACK color. UPPER PCB at mezzanine CANOpen at STM32F407
Синяя плата  STM32F4-Discovery. CANOpen с собственным OD.

Реализовано CAN-устройство на STM32F407 (EVB STM32F4DISCOVERY ).

Сеть состоит из устройств из проектов 
Disco407_Blue,

Aliex_Disco407green,

EvolutionBoard from Aliexpress BLACK color STM32F4XX LOWER PCB at mezzanine,

EvolutionBoard from Aliexpress BLACK color STM32F4XX UPPER PCB at mezzanine



CAN-устройство NodeID=103 на STM32F407 считывает запись с индексом 0x6038 из OD CAN-устройства NodeID=72 посредством SDO.
Запись имеет тип ARRAY. Считывается и перезаписывается только элемент массива с номером [0x0E].
1. Чтение с удалённого узла записи 0x6038.
2. Новое значение в  0x6038.
3. Чтение с удалённого узла записи 0x6038.

До инициализации CANOpen есть участок кода для физической проверки классического CAN-интерфейса. 
До начала передачи сообщений вводится задержка 1500ms для начала работы CAN на обоих устройствах.
Код отключен при помощи #if 0.

OD отредактирован с помощью EDSEditorGUI
Для просмотра открыть DS301_profile.xpd
Перетащить из проводника файл DS301_profile.xdd (CANopen XML Device Description File )
Так как файлы OD.h и OD.c не редактируются, а перезаписываются, то для контроля версий их следует добавлять в репозиторий при каждом commit.

Непрерывно ведётся контроль PDO с индексом x6001,6002.
В случае их изменения(с удалённого узла посредством SDO) новые значения - в терминал через UART1.

Каждые 450ms TPDO[0] передаётся на шину CAN.
