from datetime import datetime

# Huidige datum en tijd
nu = datetime.now()
formatted_datetime = nu.strftime("%d-%m-%Y_%H:%M")
print("Geformatteerde datum en tijd:", formatted_datetime)