# MCDM
## Charge and discharge module

#### Как это работает.
Зарядное устройство состоит из трех блоков. 
Первый из них блок питания на 18 вольт соответствующей мощности, например [WX-DC2406](https://aliexpress.ru/item/1005001714079538.html?_ga=2.268223694.1168692661.1643047564-769654542.1642920280&sku_id=12000018559376968&spm=a2g39.orderlist.0.0.60754aa6RxSv8u) настроенный на выходное напряжение 18 вольт.

Второй блок - плата управления на микроконтроллере ESP-WROOM-32D - дисплей, кнопки, светодиодная индикация, USB интерфейс. Назначение - ввод-вывод пользовательских настроек, исполнение выбранного алгоритма работы, логирование и управление по каналу WiFi. Данные об этом блоке в своем депозитории.

Третий блок - измерительный и зарядно-разрядный модуль. Силовая часть заимствована от [XL4016 DC-DC Max 9A 300W](https://aliexpress.ru/item/32661166685.html?_ga=2.264151500.1168692661.1643047564-769654542.1642920280&sku_id=12000020944321893&spm=a2g39.orderlist.0.0.60754aa6RxSv8u). Управляется субмодулем [WeMos D1 SAMD21 M0 Mini](https://aliexpress.ru/item/4001074203011.html?_ga=2.268755377.1168692661.1643047564-769654542.1642920280&sku_id=10000014182967685&spm=a2g39.orderlist.0.0.60754aa6RxSv8u).
Субмодуль микроконтроллера съемный, устанавливается на гиперболические [разъемы](https://aliexpress.ru/item/32358969925.html?_ga=2.23972602.1168692661.1643047564-769654542.1642920280&sku_id=58391393685&spm=a2g39.orderlist.0.0.41c74aa6suqlA3). Для работы с выходными ключами и питания субмодуля используются гальванически развязанные [источники](https://aliexpress.ru/item/1005003568781579.html?_ga=2.165478714.428003227.1651560775-769654542.1642920280&sku_id=12000026320337806&spm=a2g39.orderlist.0.0.50684aa6oqK3eu) мощностью 1 ватт. В свою очередь управляющие команды субмодуль получает по асинхронному интерфейсу от второго блока. Протокол обмена Wake.

На плате есть разъем для подключения 8-канального [логического анализатора](https://aliexpress.ru/item/4000183609972.html?spm=a2g2w.productlist.0.0.33ca2d9fLvrncQ&sku_id=10000000674073904).

Измерение напряжения на подключенной батарее производятся по 4-х контактной схеме, используя дифференциальное измерение в диапазоне от -2 до 20 вольт.

Измерение токов заряда и разряда производится одним шунтом, используется сдвиг по напряжению для возможности измерения токов разной полярности в диапазоне от -10 до +10 ампер.

Управление понижающими преобразователями производится ШИМ-сигналами частотой 190кГц. Интерфейсными схемами являются MIC4420.

Регулирование процессов производится ПИД-регуляторами. Для сокращения времени обработки вычислений все данные передаются контроллеру предвычисленными и целочисленными.

Схема разряда заимствована от Кулона 912, ток задается выходом DAC микроконтроллера. Для облегчения теплового режима предусмотрен разъем подключения внешней нагрузки.

Приведенные в этом депозитории документы относятся именно к третьему блоку зарядного устройства. 

#### Формат реализованных команд управления.

Имя | Код | Параметры | Ответ
--------------------:|--------:|----------------:|------------------:
Текущие напряж. и ток | 0x21 | нет | 0x21 0x01 0xUUUU 0xIIII 0xSSSS
 n | 0x22 |  |