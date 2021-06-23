# tp_projekt4

### projekt4.cpp
Zawiera część kodu związaną bezpośrednio z tworzeniem okna aplikacji oraz jego obsługą w trkcie dizłania programu.
#### Klasa Appstate
Zawiera zmienne opisujące aktualny stan programu oraz funkcje, które z nich korzystają.
Na przykład:
+ zmienna triangles jest wektorem wszystkich trójkątów, które są wyświetlane na ekranie
+ zmienna gravity typu PointF opisuje wektor pola grawitacyjnego
+ funkcja draw jest odpowiedzialna za rysowanie 
+ funkcja script jest odpowiedzialna za sterowanie ramieniem robota

### Triangle.h Triangle.cpp
Plik zawierający klasę Triangle

### Robot.h Robot.cpp
Plik zawierający klasę Robot

<details>
  <h3>
  <summary> UniversalConvexShape.h UniversalConexShape.cpp </summary>
  </h3>
  <p>
  Pliki zawierające klasę odzwierciadlającą dowolny wypukły kształt.
  Implementuje metody:
    -rysujące na instancji obiektu graphics
    -implementujące algorytm gjk
  </p>
</details>
