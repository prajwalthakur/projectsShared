import tensorflow as tf
#gpus = tf.config.list_physical_devices('GPU')
gpus =tf.config.experimental_list_devices()

logical_gpus = tf.config.experimental.list_logical_devices('GPU')
print(len(gpus), "Physical GPUs,", len(logical_gpus), "Logical GPUs")
