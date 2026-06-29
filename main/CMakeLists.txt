idf_component_register(SRCS "web_server.cpp" "wifi_ap.cpp" "main.cpp" "imu/imu.cpp" "motor/motor.cpp" "button/button.cpp" "potentiometer/potentiometer.cpp" "module/fastpid/fastpid.cpp"
                    INCLUDE_DIRS "." "imu" "motor" "button" "potentiometer" "module"
                    EMBED_FILES "dashboard/index.html" "dashboard/style.css" "dashboard/script.js")