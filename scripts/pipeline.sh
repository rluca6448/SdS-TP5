cmake -S . -B build
cmake --build build --config Release

./build/tp5 5 1 50 output

python3 scripts/visualizar.py output/states.txt --every 1 --fps 60 --save output/animacion4.mp4
# Estudios completos, gráficos y animaciones (ver README)
# python3 scripts/correr.py todo
# python3 scripts/graficos.py
