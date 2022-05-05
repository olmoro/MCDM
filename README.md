### <p align="center">MCDM - Moro Charge and Discharge Module
***
#### <p align="center"> Как это работает.
Зарядное устройство состоит из трех блоков. 
Первый из них блок питания на 18 вольт соответствующей мощности, например [WX-DC2406](https://aliexpress.ru/item/1005001714079538.html?_ga=2.268223694.1168692661.1643047564-769654542.1642920280&sku_id=12000018559376968&spm=a2g39.orderlist.0.0.60754aa6RxSv8u) настроенный на выходное напряжение 18 вольт.

Второй блок - плата управления на микроконтроллере ESP-WROOM-32D - дисплей, кнопки, светодиодная индикация, USB интерфейс. Назначение - ввод-вывод пользовательских настроек, исполнение выбранного алгоритма работы, логирование и управление по каналу WiFi. Данные об этом блоке в своем депозитории.

Третий блок - зарядно-разрядный модуль. Силовая часть заимствована от [XL4016 DC-DC Max 9A 300W](https://aliexpress.ru/item/32661166685.html?_ga=2.264151500.1168692661.1643047564-769654542.1642920280&sku_id=12000020944321893&spm=a2g39.orderlist.0.0.60754aa6RxSv8u). Управляется субмодулем [WeMos D1 SAMD21 M0 Mini](https://aliexpress.ru/item/4001074203011.html?_ga=2.268755377.1168692661.1643047564-769654542.1642920280&sku_id=10000014182967685&spm=a2g39.orderlist.0.0.60754aa6RxSv8u).
Субмодуль микроконтроллера съемный, устанавливается на гиперболические [разъемы](https://aliexpress.ru/item/32358969925.html?_ga=2.23972602.1168692661.1643047564-769654542.1642920280&sku_id=58391393685&spm=a2g39.orderlist.0.0.41c74aa6suqlA3). Для работы с выходными ключами и питания субмодуля используются гальванически развязанные [источники](https://aliexpress.ru/item/1005003568781579.html?_ga=2.165478714.428003227.1651560775-769654542.1642920280&sku_id=12000026320337806&spm=a2g39.orderlist.0.0.50684aa6oqK3eu) мощностью 1 ватт. В свою очередь управляющие команды субмодуль получает по асинхронному интерфейсу от платы управления.

На блоке есть разъем для подключения 8-канального [логического анализатора](https://aliexpress.ru/item/4000183609972.html?spm=a2g2w.productlist.0.0.33ca2d9fLvrncQ&sku_id=10000000674073904).

#### <p align="center">Измерение напряжения
Измерение напряжения на подключенной батарее производятся по 4-х контактной схеме, используя дифференциальное измерение в диапазоне от -1.5 до 20 вольт. Погрешность измерения __% ± __ед.мзр.

#### <p align="center">Измерение токов заряда и разряда
Измерение токов заряда и разряда производится одним шунтом, используется сдвиг по напряжению для возможности измерения токов разной полярности в диапазоне от -5 до +10 ампер (ограничивается мощностью блока питания).  Погрешность измерения __% ± __ед.мзр.

#### <p align="center">Управление разрядной схемой
Ток разряда задается 10-разрядным встроенным ЦАП с поддержанием тока ПИД-регулятором. Схема разряда заимствована от Кулона 912, ток задается выходом DAC микроконтроллера. Внешним ОУ диапазон выходного сигнала расширен до 5 вольт. Для облегчения теплового режима регулирующего MOSFET предусмотрен разъем подключения внешней нагрузки. 

#### <p align="center">Управление выходными ключами
Для реализации безискрового подключения заряжаемой батареи ключевой каскад управляется синхронно с силовым преобразователем, а именно: при отключении, например при перегрузке, одновременно отключается и силовой преобразователь. Остаточный заряд снимается подключением разрядной цепи (этот ток не проходит через токоизмерительный шунт). То есть включение преобразователя без подключения измерителя и на ХХ невозможно. Имеется возможность нагружать преобразователь при работе на холостом ходу или незначительной нагрузке. 

#### <p align="center">Регулирование мощности DC преобразователя.
Управление понижающим преобразователем производится ШИМ-сигналом частотой 190кГц посредством MIC4420. Для регулирования напряжения и токов используются ПИД-регуляторы. В режиме заряда или источника питания регуляторы напряжения и тока используют собственные настройки. Для монотонности и непрерывности выходной характеристики используется один и тот же экземпляр регулятора. Принцип переключения от поддержания тока к поддержанию напряжения и обратно соответствует аналоговому регулятору 912-го кулона. Коэффициенты допускается регулировать "на лету". Для реализации импульсных режимов предусмотрены сохранение и восстановление параметров регулирования, но задание временнЫх параметров (длительность импульсов, например) возлагается на главный контроллер. Регулирование ведется с фиксированной частотой 10 Гц. Все вычисления целочисленные, так что вычисление шага занимает лишь несколько микросекунд. С этой целью параметры управления драйверу передаются предвычисленными и целочисленными.

#### <p align="center">Интерфейс
Обмен с главным контроллером производится по асинхронному каналу на частоте 115200 бод. Протокол обмена [WAKE](https://eewiki.ru/wiki/%D0%A1%D0%BF%D0%B5%D1%86%D0%B8%D1%84%D0%B8%D0%BA%D0%B0%D1%86%D0%B8%D1%8F_%D0%BF%D1%80%D0%BE%D1%82%D0%BE%D0%BA%D0%BE%D0%BB%D0%B0_WAKE) предусматривает проверку целостности пакета и подтверждение приема. Ведущим контроллером является главный контроллер, в фоновом режиме опрашивающий драйвер и получающий пакет, содержащий измерения напряжения в милливольтах и ток в миллиамперах, а также информацию о состоянии. Управляющими командами производится включение-отключение DC источника и задание напряжения и тока для ПИД-регулятора. Предусматривается достаточное количество команд настройки АЦП, приборного смещения, калибровочных коэффициентов для  преобразования в физические величины, порогов срабатывания защиты и т.д. Для управления и настройки ПИД-регулятора, и ещё масса тестовых команд, надобность в которых практически отпала по ходу дела.

В директории Frames представлены данные для тестирования с помощью приложения WakeUsb.

#### <p align="center">Система охлаждения
Силовые элементы, [датчик температуры](https://aliexpress.ru/item/33014111002.html?spm=a2g2w.productlist.0.0.108e2a96XiZTKA&sku_id=67175460751) и [вентилятор](https://aliexpress.ru/item/4001029238958.html?spm=a2g2w.productlist.0.0.49696217XrQaE6&sku_id=10000013596178639) размещены на [радиаторе](https://aliexpress.ru/item/1005003036398261.html?_ga=2.156583358.428003227.1651560775-769654542.1642920280&sku_id=12000023368395685&spm=a2g39.orderlist.0.0.50684aa6xwye5Y) размером 100x60x10 мм. Регулирование производится ПИД-регулятором.

#### <p align="center">Печатная плата
115*65 мм двусторонняя 1oz. Схема, сборки, перечень элементов и гербер-файлы в директории Docements.

#### <p align="center">Формат реализованных команд управления.
##### <p align="center">(Не отредактировано)
Имя | Код | Параметр | Ответ | Примечание
--------------------------:|------:|-------:|-------------:|----------------------------------:
cmd_read_u_i|0x10|-|int int uint|Чтение текущего напряжения,тока (мВ и мА) и состояния
cmd_get_u|0x11|-|int|Чтение текущего напряжения (мВ)
cmd_get_i|0x12|-|int|Чтение текущего тока (мА)
cmd_get_ui|0x13|-|int int|Чтение текущего напряжения и тока (мВ и мА)
cmd_get_state|0x14|-|uint|Чтение текущего состояния
cmd_power_go|0x20|-|-|Старт преобразователя с заданными максимальными U и I
cmd_power_stop|0x21|-|-|Отключение ( и разряда в том числе)
cmd_set_pid|0x22|-|-|set all parameters (в разработке)
cmd_get_factor_u|0x30|-|uint|Чтение множителя преобразования в милливольты
cmd_set_factor_u|0x31|uint|-|Запись множителя преобразования в милливольты
cmd_set_factor_default_u|0x32|-|-|Возврат к заводскому множителю преобразования в милливольты
cmd_get_smooth_u|0x33|-|uint|Чтение параметра сглаживания по напряжению
cmd_set_smooth_u|0x34|uint|-|Запись параметра сглаживания по напряжению (1...3)
cmd_get_offset_u|0x35|-|uint|Чтение приборного смещения по напряжению
cmd_set_offset_u|0x36|int|-|Запись приборного смещения по напряжению
cmd_get_factor_i|0x38|-|uint|Чтение множителя преобразования в миллиамперы
cmd_set_factor_i|0x39|uint|-|Запись множителя преобразования в миллиамперы
cmd_set_factor_default_i|0x3A|-|-|Возврат к заводскому множителю преобразования в миллиамперы
cmd_get_smooth_i|0x3B|-|uint|Чтение параметра сглаживания по току
cmd_set_smooth_i|0x3C|uint|-|Запись параметра сглаживания по току (1...3)
cmd_get_offset_i|0x3D|-|uint|Чтение приборного смещения по току
cmd_set_offset_i|0x3E|int|-|Запись приборного смещения по току
cmd_pid_configure|0x40|uint uint uint uint uint uint|-|set mode,kp,ki,kd,min,max
cmd_pid_set_coefficients|0x41|uint uint uint|-|set kp, ki, kd
cmd_pid_output_range|0x42|uint uint|-|set min, max
cmd_pid_reconfigure|0x43|uint uint uint uint uint|-|set kp,ki,kd,min,max w/o clear
cmd_pid_clear|0x44|-|-|clear
cmd_pid_test|0x46|uint uint uint|-|mode, setpoint, sw
cmd_pwm_configure_out|0x47|?|?|    
cmd_pid_get_configure|0x48|-|uint uint uint uint uint uint|mode,kP,kI,kD,min,max - возвращает параметры текущего режима регулирования
cmd_pid_set_max_sum|0x49|uint|-|Задает максимальный интеграл при вычислении шага рег.
cmd_pwm_configure_cool|0x4A|?|-|    
cmd_adc_read_probes|0x50|-|int,int|Read all probes
cmd_adc_get_offset|0x51|-|int|Читать смещение АЦП
cmd_adc_set_offset|0x52|int|-|Запись смещения АЦП
cmd_set_switch_pin|0x54|?|-|sw_pin D4 PA14 ?
cmd_set_power|0x56|int|-|пользоваться с осторожностью - выяснение пределов регулирования
cmd_set_discharge|0x57|uint|-|не проверена
cmd_set_voltage|0x58|uint|-|старая, не проверена
cmd_set_current|0x59|uint|-|старая, не проверена 
cmd_set_discurrent|0x5A|uint|-|старая, не проверена
cmd_set_surge_compensation|0x5B|?|-|параметры подавления всплеска напряжения na
cmd_set_idle_load|0x5C|uint|-|параметры доп.нагрузки ХХ
cmd_get_win_less_u|0x60|-|int|Порог отключения по Umin 
cmd_set_win_less_u|0x61|int|-|Порог отключения по Umin
cmd_set_win_less_default_u|0x62|int|-|Порог отключения по Umin 
cmd_get_win_up_u|0x63|-|int|Порог отключения по Umax
cmd_set_win_up_u|0x64|int|-|Порог отключения по Umax
cmd_set_win_up_default_u|0x65|int|-|Порог отключения по Umax 
cmd_get_win_less_i|0x68|-|int|Порог отключения по Imin 
cmd_set_win_less_i|0x69|int|-|Порог отключения по Imin 
cmd_set_win_less_default_i|0x6A|int|-|Порог отключения по Imin 
cmd_get_win_up_i|0x6B|-|int|Порог отключения по Imax 
cmd_set_win_up_i|0x6C|int|-|Порог отключения по Imax  
cmd_set_win_up_default_i|0x6D|int|-|Порог отключения по Imax 
