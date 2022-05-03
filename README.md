# MCDM
## Charge and discharge module

#### Как это работает.
Зарядное устройство состоит из трех блоков. 
Первый из них блок питания на 18 вольт соответствующей мощности, например [WX-DC2406](https://aliexpress.ru/item/1005001714079538.html?_ga=2.268223694.1168692661.1643047564-769654542.1642920280&sku_id=12000018559376968&spm=a2g39.orderlist.0.0.60754aa6RxSv8u) настроенный на выходное напряжение 18 вольт.

Второй блок - плата управления на микроконтроллере ESP-WROOM-32D - дисплей, кнопки, светодиодная индикация, USB интерфейс. Назначение - ввод-вывод пользовательских настроек, исполнение выбранного алгоритма работы, логирование и управление по каналу WiFi. Данные об этом блоке в своем депозитории.

Третий блок - измерительный и зарядно-разрядный модуль. Силовая часть заимствована от [XL4016 DC-DC Max 9A 300W](https://aliexpress.ru/item/32661166685.html?_ga=2.264151500.1168692661.1643047564-769654542.1642920280&sku_id=12000020944321893&spm=a2g39.orderlist.0.0.60754aa6RxSv8u). Управляется субмодулем [WeMos D1 SAMD21 M0 Mini](https://aliexpress.ru/item/4001074203011.html?_ga=2.268755377.1168692661.1643047564-769654542.1642920280&sku_id=10000014182967685&spm=a2g39.orderlist.0.0.60754aa6RxSv8u).
Субмодуль микроконтроллера съемный, устанавливается на гиперболические [разъемы](https://aliexpress.ru/item/32358969925.html?_ga=2.23972602.1168692661.1643047564-769654542.1642920280&sku_id=58391393685&spm=a2g39.orderlist.0.0.41c74aa6suqlA3). Для работы с выходными ключами и питания субмодуля используются гальванически развязанные [источники](https://aliexpress.ru/item/1005003568781579.html?_ga=2.165478714.428003227.1651560775-769654542.1642920280&sku_id=12000026320337806&spm=a2g39.orderlist.0.0.50684aa6oqK3eu) мощностью 1 ватт. В свою очередь управляющие команды субмодуль получает по асинхронному интерфейсу от второго блока. Протокол обмена Wake.

На плате есть разъем для подключения 8-канального [логического анализатора](https://aliexpress.ru/item/4000183609972.html?spm=a2g2w.productlist.0.0.33ca2d9fLvrncQ&sku_id=10000000674073904).

#### Измерение напряжения
Измерение напряжения на подключенной батарее производятся по 4-х контактной схеме, используя дифференциальное измерение в диапазоне от -1.5 до 20 вольт.

#### Измерение токов заряда и разряда
Измерение токов заряда и разряда производится одним шунтом, используется сдвиг по напряжению для возможности измерения токов разной полярности в диапазоне от -5 до +16 ампер.

#### Управление разрядной схемой

Ток разряда задается 10-разрядным встроенным ЦАП с поддержанием тока ПИД-регулятором. Схема разряда заимствована от Кулона 912, ток задается выходом DAC микроконтроллера. Внешним ОУ диапазон выходного сигнала расширен до 5 вольт. Для облегчения теплового режима регулирующего MOSFET предусмотрен разъем подключения внешней нагрузки. 

#### Управление выходными ключами

Для реализации безискрового подключения заряжаемой батареи ключевой каскад управляется синхронно с силовым преобразователем, а именно: при отключении, например при перегрузке, одновременно отключается и силовой преобразователь. Остаточный заряд снимается подключением разрядной цепи (этот ток не проходит через токоизмерительный шунт). То есть включение преобразователя без подключения измерителя и на ХХ невозможно. Имеется возможность нагружать преобразователь при работе на холостом ходу или незначительной нагрузке. 

#### Регулирование мощности DC преобразователя.

Управление понижающим преобразователем производится ШИМ-сигналом частотой 190кГц посредством MIC4420. Для регулирования напряжения и токов используются ПИД-регуляторы. В режиме заряда или источника питания регуляторы напряжения и тока используют собственные настройки. Для монотонности и непрерывности выходной характеристики используется один и тот же экземпляр регулятора. Принцип переключения от поддержания тока к поддержанию напряжения и обратно соответствует аналоговому регулятору 912-го кулона. Коэффициенты допускается регулировать "на лету". Для реализации импульсных режимов предусмотрены сохранение и восстановление параметров регулирования, но задание временнЫх параметров (длительность импульсов, например) возлагается на главный контроллер. Регулирование ведется с фиксированной частотой 10 Гц. Все вычисления целочисленные, так что вычисление шага занимает лишь несколько микросекунд. С этой целью параметры управления драйверу передаются предвычисленными и целочисленными.

#### Интерфейс

Обмен с главным контроллером производится по асинхронному каналу на частоте 115200 бод. Протокол обмена [WAKE](https://eewiki.ru/wiki/%D0%A1%D0%BF%D0%B5%D1%86%D0%B8%D1%84%D0%B8%D0%BA%D0%B0%D1%86%D0%B8%D1%8F_%D0%BF%D1%80%D0%BE%D1%82%D0%BE%D0%BA%D0%BE%D0%BB%D0%B0_WAKE) предусматривает проверку целостности пакета и подтверждение приема. Ведущим контроллером является главный контроллер, в фоновом режиме опрашивающий драйвер и получающий пакет, содержащий измерения напряжения в милливольтах и ток в миллиамперах, а также информацию о состоянии. Управляющими командами производится включение-отключение DC источника и задание напряжения и тока для ПИД-регулятора. Предусматривается достаточное количество команд настройки АЦП, приборного смещения, калибровочных коэффициентов для  преобразования в физические величины, порогов срабатывания защиты и т.д. Для управления - настройки ПИД-регулятора и ещё масса тестовых команд, надобность в которых практически отпала по ходу дела.

#### Система охлаждения
Силовые элементы, [датчик температуры](https://aliexpress.ru/item/33014111002.html?spm=a2g2w.productlist.0.0.108e2a96XiZTKA&sku_id=67175460751) и [вентилятор](https://aliexpress.ru/item/4001029238958.html?spm=a2g2w.productlist.0.0.49696217XrQaE6&sku_id=10000013596178639) размещены на [радиаторе](https://aliexpress.ru/item/1005003036398261.html?_ga=2.156583358.428003227.1651560775-769654542.1642920280&sku_id=12000023368395685&spm=a2g39.orderlist.0.0.50684aa6xwye5Y) размером 100x60x10 мм. Регулирование производится ПИД-регулятором.

#### Печатная плата
115*65мм двусторонняя 1oz. Схема, сборки, перечень элементов и гербер-файлы в директории Docements.

#### Формат реализованных команд управления.

Имя | Код | Параметры | Ответ
--------------------:|--------:|----------------:|------------------:
Текущие напряж. и ток | 0x21 | нет | 0x21 0x01 0xUUUU 0xIIII 0xSSSS
 n | 0x22 |  |