#include "vision_ov_yolo/yolo_engine.hpp"

// Daftar nama kelas hardcoded (bisa dipindah ke config sebenarnya)
const std::vector<std::string> CLASS_NAMES = {"Baskom", "Flare"};

YoloEngine::YoloEngine(const std::string& model_path, float conf_thr, float nms_thr, float score_thr) 
    : conf_threshold_(conf_thr), nms_threshold_(nms_thr), score_threshold_(score_thr) {
    
    // Load Model Logic
    std::shared_ptr<ov::Model> model = core_.read_model(model_path);
    
    // Preprocessing setup using OpenVINO API 2.0 (Modern Way)
    ov::preprocess::PrePostProcessor ppp(model);
    ppp.input().tensor()
        .set_element_type(ov::element::u8)
        .set_layout("NHWC")
        .set_color_format(ov::preprocess::ColorFormat::RGB);
        
    ppp.input().preprocess()
        .convert_element_type(ov::element::f32)
        .convert_color(ov::preprocess::ColorFormat::RGB)
        .scale({255.0f, 255.0f, 255.0f});

    ppp.input().model().set_layout("NCHW");
    ppp.output().tensor().set_element_type(ov::element::f32);
    
    model = ppp.build();
    compiled_model_ = core_.compile_model(model, "CPU");
    infer_request_ = compiled_model_.create_infer_request();
}

void YoloEngine::preprocess(const cv::Mat& frame, float& ratio, int& pad_w, int& pad_h) {
    // Letterbox resize logic
    float scale = std::min((float)INPUT_W / frame.cols, (float)INPUT_H / frame.rows);
    int new_unpad_w = std::round(frame.cols * scale);
    int new_unpad_h = std::round(frame.rows * scale);
    
    pad_w = (INPUT_W - new_unpad_w) / 2;
    pad_h = (INPUT_H - new_unpad_h) / 2;
    ratio = scale;

    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(new_unpad_w, new_unpad_h));
    
    // Canvas abu-abu
    cv::Mat canvas(cv::Size(INPUT_W, INPUT_H), CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(canvas(cv::Rect(pad_w, pad_h, new_unpad_w, new_unpad_h)));

    // Konversi BGR -> RGB untuk input tensor
    cv::Mat rgb_frame;
    cv::cvtColor(canvas, rgb_frame, cv::COLOR_BGR2RGB);
    
    // Set input tensor (Zero-copy approach jika memungkinkan, tapi ini aman)
    input_tensor_ = infer_request_.get_input_tensor();
    // Copy data mat ke tensor pointer
    std::memcpy(input_tensor_.data(), rgb_frame.data, INPUT_W * INPUT_H * 3);
}

std::vector<BBox> YoloEngine::run_inference(cv::Mat& frame) {
    float ratio;
    int dw, dh;
    preprocess(frame, ratio, dw, dh);

    infer_request_.infer();

    const auto& output_tensor = infer_request_.get_output_tensor();
    const ov::Shape output_shape = output_tensor.get_shape();
    const float* detection_data = output_tensor.data<const float>();

    return postprocess(detection_data, output_shape, frame.size(), ratio, dw, dh);
}

std::vector<BBox> YoloEngine::postprocess(const float* data, const ov::Shape& shape, const cv::Size& orig_size, float ratio, int dw, int dh) {
    std::vector<cv::Rect> boxes;
    std::vector<int> class_ids;
    std::vector<float> confidences;

    size_t rows = shape[1]; // 25200 anchors biasanya
    size_t dimensions = shape[2]; // 5 + N_CLASSES

    for (size_t i = 0; i < rows; ++i) {
        const float* current_det = data + i * dimensions;
        float objectness = current_det[4];

        if (objectness < 0.1f) continue; // Filter awal

        float max_score = 0.0f;
        int best_class_id = -1;

        // Mencari class score tertinggi
        for (size_t j = 5; j < dimensions; ++j) {
            if (current_det[j] > max_score) {
                max_score = current_det[j];
                best_class_id = j - 5;
            }
        }

        float final_conf = objectness * max_score;
        if (final_conf >= score_threshold_) {
            float cx = current_det[0];
            float cy = current_det[1];
            float w = current_det[2];
            float h = current_det[3];

            // Kembalikan ke koordinat asli (Inverse Letterbox)
            int x = (cx - w * 0.5f - dw) / ratio;
            int y = (cy - h * 0.5f - dh) / ratio;
            int width = w / ratio;
            int height = h / ratio;

            boxes.emplace_back(x, y, width, height);
            class_ids.push_back(best_class_id);
            confidences.push_back(final_conf);
        }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

    std::vector<BBox> results;
    for (int idx : indices) {
        BBox res;
        res.box = boxes[idx] & cv::Rect(0, 0, orig_size.width, orig_size.height); // Clip box
        res.class_id = class_ids[idx];
        res.confidence = confidences[idx];
        if (res.class_id < (int)CLASS_NAMES.size()) {
            res.label = CLASS_NAMES[res.class_id];
        } else {
            res.label = "Unknown";
        }
        results.push_back(res);
    }
    return results;
}