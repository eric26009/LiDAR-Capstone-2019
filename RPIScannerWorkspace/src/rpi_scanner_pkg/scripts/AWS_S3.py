import boto3
import datetime

f = open("/home/pi/Documents/AWS_key.txt", "r")
AWS_ACCESS_KEY_ID = f.readline().splitlines()[0]
AWS_SECRET_ACCESS_KEY = f.readline().splitlines()[0]
print("AWS_ACCESS_KEY_ID", AWS_ACCESS_KEY_ID)
print("AWS_SECRET_ACCESS_KEY", AWS_SECRET_ACCESS_KEY)

# creating a client to access s3 service
s3 = boto3.client('s3', aws_access_key_id=AWS_ACCESS_KEY_ID, aws_secret_access_key=AWS_SECRET_ACCESS_KEY)


def upload(fileName, s3folder, name):
    """Accepts a string filename/path and will upload it to S3 with date/time appended"""
    date_append = str(datetime.datetime.now()).replace(" ", "")
    extension = fileName[fileName.find("."):len(fileName)]
    s3.upload_file(fileName, "uw-capstone2019", s3folder + "/" + name+extension)
    print(s3folder + "/" + name)
    return name

def download(fileName, s3folder):
    """Downloads file from s3"""
    s3.download_file("uw-capstone2019", s3folder + "/" + fileName, fileName)


#upload("test.bag","Bag_files","atestbag")
#download("testfile_2019-07-18 11:28:38.883803.txt", "Bag_files")
