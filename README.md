# Nawigacja kołowego robota mobilnego z wykorzystaniem czujników zbliżeniowych

Głównym celem projektu dyplomowego było stworzenie algorytmu sterowania robotem 
mobilnym. Algorytm ten ma umożliwiać robotowi nawigację z wykorzystaniem czujników 
zbliżeniowych. Robot ma dojechać do celu w nieznanym środowisku, mapując na bieżąco przeszkody, by je ominąć. Głównym założeniem projektu, było przystosowanie sterowania dla przypadku, w którym część czujników nie działa. W skrajnej sytuacji sterownik jest w stanie nawigować robota, nawet z wykorzystaniem tylko jednego czujnika. Sterownik napisano w języku C++, można go znaleść w *simulation/controllers/ctrl_1/ctrl_1.cpp*.
Do jego stworzenia wykorzystano środowisko symulacyjne Webots, a następnie przetestowano na fizycznym robocie. 
