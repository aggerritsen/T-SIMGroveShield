pio run -t clean -e t-sim7000g
pio run -e t-sim7000g -t upload; if ($LASTEXITCODE -eq 0) { pio device monitor -p COM13 }
