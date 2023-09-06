#include <pgmspace.h>
 
#define SECRET
#define THINGNAME "ESP32_DHT11"                         //change this
 
const char WIFI_SSID[] = "NOKIA-01D1";               //change this
const char WIFI_PASSWORD[] = "CmSem67iiD";           //change this
const char AWS_IOT_ENDPOINT[] = "a3v4vm8d91hy76-ats.iot.ap-southeast-1.amazonaws.com";       //change this
 
// Amazon Root CA 1
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";
 
// Device Certificate                                               //change this
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIURgkb+OhE2LiYohsaH3br0WFB4CIwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTIzMDUwNTE0MDE0
N1oXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBANc4ngpW/kKgsa8eX5ap
Ko+ryhSonexFJB6ox6RSYW8gox9uHt2/BuEQhjLbQCiExVmhfY0o1/DGR3/oT6Ex
y3+8uUHO48I3dOlFj7CcZEeVwzYwbt+85W/gJVqiCOF0Qmd5CJqwG8ljwfO9/BHY
CR3zQOiOn8XF8A0sV6AR7vjhSjleabtCoJABlOFkN+urx9yq1lthQevQbhsXijoB
u9IEpzhy6/H3AMpFCwKbv/iGosGja01znXhCyNWp5C2HbtCEid5DbXjrNp79NvZc
tMHJxVEBN1Q+gT//1CjkzLYbGMeF5VdrouO8NEfkfbq0REKVdtaUuMdGg74MUqSm
Dd8CAwEAAaNgMF4wHwYDVR0jBBgwFoAUi/Sna4RUXpvdzDD6Twy+yVdNL7MwHQYD
VR0OBBYEFHupEcVLgkjn3TYshhcocPEs9LnoMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQBfH9y6EOGZWJ03pqYQGuO+Dujm
dVX/g0spzTeN0Acf2jAx+z1nOnQVL+Psgp4vdgjpgjUCuVpfG04ZrWPxk3sWul66
719Y9cyM7fSK+Jnfns0LMiq/X01UOxg3bBu5U7/m37lUHsGs7n6XijGTDAdXtRvC
Lw2CX1kxXzoOnQvnzgNrShSZnNkYCM6K9kmiFzJyWHGIe/hoLJ3Ym7fZAetzvGC+
y+NlIgqfQb+tsb3VVb85Bv4xImJUBrl/kWZirBLO9MNHfgs2WzYmcWVDLRZjalDH
aGsjbOC+LMpjx+Zkzp6RzBJQZBC83xI7jaVhM7xLV5I72zcE/KqsuzxdGMdn
-----END CERTIFICATE-----
 
 
)KEY";
 
// Device Private Key                                               //change this
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEpQIBAAKCAQEA1zieClb+QqCxrx5flqkqj6vKFKid7EUkHqjHpFJhbyCjH24e
3b8G4RCGMttAKITFWaF9jSjX8MZHf+hPoTHLf7y5Qc7jwjd06UWPsJxkR5XDNjBu
37zlb+AlWqII4XRCZ3kImrAbyWPB8738EdgJHfNA6I6fxcXwDSxXoBHu+OFKOV5p
u0KgkAGU4WQ366vH3KrWW2FB69BuGxeKOgG70gSnOHLr8fcAykULApu/+IaiwaNr
TXOdeELI1ankLYdu0ISJ3kNteOs2nv029ly0wcnFUQE3VD6BP//UKOTMthsYx4Xl
V2ui47w0R+R9urREQpV21pS4x0aDvgxSpKYN3wIDAQABAoIBAQCoJGqMR3Szlxna
Z49khFtlDWB+jVF74nSao1/9yAKpMEVg68aflXuJYECIAPT58ui/4sPVAaDbUf1Q
N63mkKGc9VOCz2DvPiZPjPH6l+RcbOurLWGh2NEje2mTil4Hza2Ha27Gc2nyB8bi
Z35EbpSI2p/OUIcQm1i04fy/aONbei/pc6K7QDPppbW0C8gVvLuskuyNPDkYO7/3
sDIwCjrjsQRnKySUWd/8t0NwC0Uw4Bs4EPa0Nz+VP1yvy15dw/nYVp0rOwfNW1sA
XYxgQHRRYP5a3j/JNwcIwrK9dO34qBFhxJDxAzSvxrhYJ51MS6+l3XGAZMVEWYGv
I6JKzzMhAoGBAPnNVmm/wjG3pPqkuq+10KXlMZfFbvw+b6InbTzblkSMkxvVFAEa
qh6vBRGnjG1bkMnQFgHyryJcfObXAjp+O5Amm+7VcOFcQWnZ0RMZeBAoYYUHQz53
VsfswIZCNW6VBjs+38Jt22a4+iPaOKXxA0GI7YiusixhggCFMsNXTkrPAoGBANyP
ogBCC684O1ON991pPvLcb6du0XnmIf3tvArhApkqfplILv6aaMQYFW3aFXYfKAQX
hAcitXZSM8LPQCYoGjAM8HugN+9nMOfLy2c8z+aIpqf/KRd9cB65aWA//rQ5gDaM
QRCW/ooUoad54FCLw0K0ymXVk0BtrHFhEo0vWI/xAoGBAKdWdHK2sGIZwXzd6OlY
BIN9O3GOIsD7GlEn1DeWad1Vxh446QOsVxXi7KRc9uczFZT7viJj7go4prsEgUup
0EDKfF7LwKhW4sMD1NjUkOdSBbHVoagwzYN0B6xood6IGfqKLd7TNMxlUbu+rYAM
KcA299g+HsE4OJwcnlrZG2tlAoGBAIY9pYwippnuz6z7K3FHLVkDcbGmcTiQKQA0
Qc8dREeU5ZjFplSwBjPTtLc3lPhf3isfZOwy498wCPD400JmBgpY1cVorqK3W7y4
/QPIr4DQix6U3p29axBIKSq7yHpIxRBuLOu7+xBvnjQgqbZcWmuvPcDdTNJZ7irT
ukbUGkMBAoGAakIT/XYPUu98+JaHUP3/d9png2WxVlIAtSot1MqUgksI0PL4Qe8y
Q+V3EtE0jw97y6jlMVBfyD5A4jw2UusdWFOb8nbI43p7P4u8irec0Sd7cYL1TirL
eJA9ZKpV9Ab8yUcBxwApkHb9+jvjn6GPRUgiRWPwli9MR5v8ha9nNHI=
-----END RSA PRIVATE KEY-----
 
 
)KEY";