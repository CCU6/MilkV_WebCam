#coding:utf-8
from ultralytics import YOLO

# 加载预训练模型
# 添加注意力机制，SEAtt_yolov8.yaml 默认使用的是n。
# SEAtt_yolov8s.yaml，则使用的是s，模型。
model = YOLO("ultralytics/cfg/models/v8/yolov8s.yaml").load('yolov8s.pt')

# Use the model
if __name__ == '__main__':
    # Use the model
    results = model.train(data='../mydataset.yaml', epochs=300, batch=4)  # 训练模型
    # 将模型转为onnx格式
    # success = model.export(format='onnx')


