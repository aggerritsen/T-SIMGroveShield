pio run -t clean -e t-sim7670g-s3
pio run -e t-sim7670g-s3 -t upload; if ($LASTEXITCODE -eq 0) { pio device monitor -p COM3 }
