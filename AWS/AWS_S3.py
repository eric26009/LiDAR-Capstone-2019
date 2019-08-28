import boto3
import datetime

f = open("/home/pi/Documents/AWS_key.txt", "r")
AWS_ACCESS_KEY_ID = f.readline().splitlines()[0]
AWS_SECRET_ACCESS_KEY = f.readline().splitlines()[0]
print("AWS_ACCESS_KEY_ID", AWS_ACCESS_KEY_ID)
print("AWS_SECRET_ACCESS_KEY", AWS_SECRET_ACCESS_KEY)

# creating a client to access s3 service
s3 = boto3.client('s3', aws_access_key_id=AWS_ACCESS_KEY_ID, aws_secret_access_key=AWS_SECRET_ACCESS_KEY)


def upload(fileName, s3folder):
    """Accepts a string filename/path and will upload it to S3 with date/time appended"""
    date_append = str(datetime.datetime.now())
    extension = fileName[fileName.find("."):len(fileName)]
    s3.upload_file(fileName, "uw-capstone2019", s3folder + "/" + fileName[0: fileName.find(".")] + "_" +date_append+extension)
    print(s3folder + "/" + fileName[0: fileName.find(".")] + "_" +date_append+extension)

def download(fileName, s3folder):
    """Downloads file from s3"""
    s3.download_file("uw-capstone2019", s3folder + "/" + fileName, fileName)

def listObjectsInFolder(s3folder):
    objects = s3.list_objects(Bucket = "uw-capstone2019", Prefix = "Snapshot_PCD")
    print objects



# upload("testfile.txt","Bag_files")
# download("snapshot_2019-07-18 15:50:55.643764.pcd", "Snapshot_PCD")
listObjectsInFolder("Snapshot_PCD")
