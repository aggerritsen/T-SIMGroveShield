pio run -t clean -e t-sim7600
pio run -e t-sim7600 -t upload; if ($LASTEXITCODE -eq 0) { pio device monitor -p COM3 }
