def get_model_config_fc():

    # handle purely non-visual observations with a simple fully-connected network
    fc_hidden_layer_sizes = [512]
    fc_activation = "relu"

    model_config_fc = {
        "fcnet_hiddens": fc_hidden_layer_sizes,
        "fcnet_activation": fc_activation
    }

    return model_config_fc


def get_model_config_conv(image_height, image_width):

    #
    # handle mixed observations with a simple network architecture inspired by my Stanford CS231n Problem Set 2 reference implementation
    #
    # in_channel  = 3
    # channel_1   = 32
    # channel_2   = 32
    # channel_3   = 32
    # channel_4   = 32
    # num_classes = 10
    #    
    # KH1,KW1 = 3,3
    # KH2,KW2 = 3,3
    # KH3,KW3 = 3,3
    # KH4,KW4 = 3,3
    # H,W     = 32,32
    #
    # nn.Conv2d(in_channel, channel_1, (KH1,KW1), padding=1, bias=True)
    # nn.ReLU()
    # nn.Conv2d(channel_1,  channel_2, (KH2,KW2), padding=1, bias=True)
    # nn.ReLU()
    # nn.MaxPool2d(2,2)
    # nn.Conv2d(channel_2,  channel_3, (KH3,KW3), padding=1, bias=True)
    # nn.ReLU()
    # nn.Conv2d(channel_3,  channel_4, (KH4,KW4), padding=1, bias=True)
    # nn.ReLU()
    # nn.MaxPool2d(2,2)
    # Flatten()
    # nn.Linear(channel_4*int(H/4)*int(W/4), num_classes)
    #

    conv_num_channels_per_conv_layer = 32
    conv_activation = "relu"
    conv_num_pool_layers = 2
    conv_num_channels_final_conv_layer = 512
    conv_fc_hidden_layer_sizes = []

    # make sure that our image size matches the network architecture
    assert image_height/2**conv_num_pool_layers == int(image_height/2**conv_num_pool_layers)
    assert image_width/2**conv_num_pool_layers  == int(image_width/2**conv_num_pool_layers)

    # format of conv_filters is a list of [out_channels, kernel, stride]
    conv_filters = []
    for i in range(conv_num_pool_layers):
        conv_filters.append([conv_num_channels_per_conv_layer, [3, 3], 1]) # Conv2d (followed implicitly by ReLU)
        conv_filters.append([conv_num_channels_per_conv_layer, [3, 3], 1]) # Conv2d (followed implicitly by ReLU)
        conv_filters.append([conv_num_channels_per_conv_layer, [2, 2], 2]) # Pool

    # Flatten
    conv_filters.append([
        conv_num_channels_final_conv_layer,
        [int(image_height/2**conv_num_pool_layers), int(image_width/2**conv_num_pool_layers)],
        1])

    model_config_conv = {
        "conv_filters": conv_filters,
        "conv_activation": conv_activation,
        "post_fcnet_hiddens": conv_fc_hidden_layer_sizes,
        "post_fcnet_activation": None
    }

    return model_config_conv
