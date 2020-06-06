import sys
import gspread
from oauth2client.service_account import ServiceAccountCredentials
import datetime

# This script must be passed two arguments: CVT60 unit number and the result of the feeding
unit = sys.argv[1]
result = sys.argv[2]

scope = ['https://spreadsheets.google.com/feeds','https://www.googleapis.com/auth/drive']
creds = ServiceAccountCredentials.from_json_keyfile_name('CVT60 logger-930552f40412.json', scope)
client = gspread.authorize(creds)

for attempt in range(10):
    try:
        sheet = client.open('CVT60 log').sheet1

        date = str(datetime.datetime.now())

        row = [date[:10], date[11:19], unit, result]
        index = 6   # Report data to be inserted in row 6
        sheet.insert_row(row, index)
        break

    except:
        print('Cannot connect to GoogleDrive')

