idf_component_register(SRCS "web_server.cpp" "wifi_ap.cpp" "main.cpp" "imu/imu.cpp" "motor/motor.cpp" "button/button.cpp" "potentiometer/potentiometer.cpp"
                    INCLUDE_DIRS "."
                    EMBED_FILES "dashboard/index.html" "dashboard/style.css" "dashboard/script.js")