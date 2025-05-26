# Nawigacja kołowego robota mobilnego z wykorzystaniem czujników zbliżeniowych

Głównym celem projektu inżynierskiego było stworzenie algorytmu sterowania dla robota mobilnego, który umożliwia autonomiczną nawigację w nieznanym środowisku z wykorzystaniem czujników zbliżeniowych.

Robot porusza się w kierunku wyznaczonego celu, dynamicznie tworząc mapę przeszkód na podstawie bieżących odczytów sensorów i omijając napotkane bariery. Szczególnym założeniem projektu było zaprojektowanie algorytmu odpornego na uszkodzenia — sterownik potrafi prowadzić robota nawet w przypadku niedostępności większości czujników, a w skrajnym przypadku działa nawet z wykorzystaniem jednego sensora.

Sterownik został zaimplementowany w języku C++ i znajduje się w pliku:

```
simulation/controllers/ctrl_1/ctrl_1.cpp
```

Do budowy i testowania wykorzystano środowisko symulacyjne **Webots**, a następnie algorytm został sprawdzony na rzeczywistym robocie.

