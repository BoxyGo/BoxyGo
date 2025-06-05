#!/usr/bin/env bash

# Ścieżka do katalogu z plikami launch w pakiecie boxygo
LAUNCH_DIR="$HOME/BoxyGo/src/boxygo/launch"

# Sprawdzenie, czy katalog istnieje
if [[ ! -d "$LAUNCH_DIR" ]]; then
  echo "Błąd: katalog '$LAUNCH_DIR' nie istnieje."
  exit 1
fi

# Przejście do katalogu z plikami launch
cd "$LAUNCH_DIR" || { echo "Nie można przejść do '$LAUNCH_DIR'"; exit 1; }

# Wczytanie do tablicy wszystkich plików *.py (walidujemy rozszerzenie .py)
mapfile -t LISTA_PLIKOW < <(ls -1 *.py 2>/dev/null)

# Jeśli nie ma żadnego pliku .py, kończymy
if [[ ${#LISTA_PLIKOW[@]} -eq 0 ]]; then
  echo "Brak plików '.py' w katalogu '$LAUNCH_DIR'."
  exit 1
fi

# Wyświetlenie ponumerowanej listy tylko tych plików .py
echo "Dostępne pliki .py w '$LAUNCH_DIR':"
for i in "${!LISTA_PLIKOW[@]}"; do
  NUMERO=$((i + 1))
  echo "  $NUMERO) ${LISTA_PLIKOW[$i]}"
done

# Pytanie użytkownika o wybór
echo -n "Wybierz numer pliku do uruchomienia (1-${#LISTA_PLIKOW[@]}): "
read WYBOR

# Walidacja: czy to liczba
if ! [[ "$WYBOR" =~ ^[0-9]+$ ]]; then
  echo "Błąd: podana wartość '$WYBOR' nie jest liczbą."
  exit 1
fi

# Walidacja: czy w zakresie
if (( WYBOR < 1 || WYBOR > ${#LISTA_PLIKOW[@]} )); then
  echo "Błąd: numer poza zakresem (1–${#LISTA_PLIKOW[@]})."
  exit 1
fi

# Pobranie wybranego pliku
INDEX=$((WYBOR - 1))
WYBRANY_PLIK="${LISTA_PLIKOW[$INDEX]}"

# Dodatkowa walidacja rozszerzenia (choć lista już filtruje .py):
if [[ "$WYBRANY_PLIK" != *.py ]]; then
  echo "Błąd: wybrany plik '$WYBRANY_PLIK' nie ma rozszerzenia .py."
  exit 1
fi

# Uruchomienie ros2 launch
echo "Uruchamiam: ros2 launch boxygo $WYBRANY_PLIK"
exec ros2 launch boxygo "$WYBRANY_PLIK"
