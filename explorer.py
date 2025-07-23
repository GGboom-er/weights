import subprocess
from functools import partial


def run(url):
    command = r'"C:/Program Files/Internet Explorer/iexplore.exe" {url}'.format(url=url)
    subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)


help = partial(run, "https://www.bilibili.com/video/av92491279/")
download = partial(run, "https://afdian.net/p/7de5199e235a11edb58252540025c377")
study = partial(run, "https://www.aboutcg.org/courseDetails/808/introduce")
