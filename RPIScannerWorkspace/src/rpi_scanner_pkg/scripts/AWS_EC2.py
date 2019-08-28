import boto3
from botocore.exceptions import ClientError

f = open("/home/pi/Documents/AWS_key.txt", "r")
AWS_ACCESS_KEY_ID = f.readline().splitlines()[0]
AWS_SECRET_ACCESS_KEY = f.readline().splitlines()[0]
AWS_INSTANCE_ID = f.readline().splitlines()[0]

# creating a client to access EC2 service
ec2 = boto3.client('ec2', aws_access_key_id=AWS_ACCESS_KEY_ID, aws_secret_access_key=AWS_SECRET_ACCESS_KEY, region_name="us-east-1")


def ec2_stop():
    try:
        ec2.stop_instances(InstanceIds=[AWS_INSTANCE_ID], DryRun=False)
        print("EC2 stopped!")
    except ClientError as e:
        print("Error", e)


def ec2_start():
    try:
        ec2.start_instances(InstanceIds=[AWS_INSTANCE_ID], DryRun=False)
        print("EC2 launched!")
    except ClientError as e:
        print("Error", e)


def ec2_reboot():
    try:
        ec2.reboot_instances(InstanceIds=[AWS_INSTANCE_ID], DryRun=False)
        print("Reboot started")
    except ClientError as e:
        print('Error', e)

# will return 0 for stopped, 1 for running, 2 for transation state
def checkRunning():
    response = ec2.describe_instances()
    for reservation in response["Reservations"]:
        for instance in reservation["Instances"]:
            instance = (instance["InstanceId"], instance["State"])
            state = instance[1]
            if instance[0] == 'i-0930f522822a57340':
                print "The instance i-0930f522822a57340 is", state["Name"]
                if state["Name"] == "running":
                    return 1
                elif state["Name"] == "stopped":
                    return 0
                else:
                    return 2


print checkRunning()
# start("i-01b87c3de7231051f")
# stop("i-01b87c3de7231051f")
# reboot("i-01b87c3de7231051f")
