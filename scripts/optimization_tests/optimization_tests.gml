if (os_get_config() == "performance_test") {
    #region quat stuff
    var t = get_timer();
    repeat 1_000_000 {
        smf_dq_create(1, 2, 3, 4, 5, 6, 7);
    }
    show_debug_message($"base dq create: {(get_timer() - t) / 1000} ms");
    
    t = get_timer();
    repeat 1_000_000 {
        smf_dq_create_opt(1, 2, 3, 4, 5, 6, 7);
    }
    show_debug_message($"optimized dq create: {(get_timer() - t) / 1000} ms");
    
    var dq = array_create(8);
    var mat = matrix_build(0, 0, 0, 30, 45, 90, 1, 1, 1);
    
    t = get_timer();
    repeat 1_000_000 {
        smf_dq_create_from_matrix(mat, dq);
    }
    show_debug_message($"base dq from matrix: {(get_timer() - t) / 1000} ms");
    
    t = get_timer();
    repeat 1_000_000 {
        smf_dq_create_from_matrix_opt(mat, dq);
    }
    show_debug_message($"optimized dq from matrix: {(get_timer() - t) / 1000} ms");
    
    t = get_timer();
    repeat 1_000_000 {
        smf_dq_duplicate(dq);
    }
    show_debug_message($"base dq clone: {(get_timer() - t) / 1000} ms");
    
    t = get_timer();
    repeat 1_000_000 {
        smf_dq_duplicate_opt(dq);
    }
    show_debug_message($"optimized dq clone: {(get_timer() - t) / 1000} ms");
    
    t = get_timer();
    repeat 1_000_000 {
        smf_dq_get_translation(dq);
    }
    show_debug_message($"base dq get translation: {(get_timer() - t) / 1000} ms");
    
    t = get_timer();
    repeat 1_000_000 {
        smf_dq_get_translation_opt(dq);
    }
    show_debug_message($"optimized dq get translation: {(get_timer() - t) / 1000} ms");
    
    t = get_timer();
    repeat 1_000_000 {
        smf_dq_get_x(dq);
    }
    show_debug_message($"base dq get single axis: {(get_timer() - t) / 1000} ms");
    
    t = get_timer();
    repeat 1_000_000 {
        smf_dq_get_x_opt(dq);
    }
    show_debug_message($"optimized dq get single axis: {(get_timer() - t) / 1000} ms");
    
    t = get_timer();
    repeat 1_000_000 {
        smf_dq_multiply(dq, dq);
    }
    show_debug_message($"base dq multiply: {(get_timer() - t) / 1000} ms");
    
    t = get_timer();
    repeat 1_000_000 {
        smf_dq_multiply_opt(dq, dq);
    }
    show_debug_message($"optimized dq multiply: {(get_timer() - t) / 1000} ms");
    
    t = get_timer();
    repeat 1_000_000 {
        smf_dq_normalize(dq);
    }
    show_debug_message($"base dq normalize: {(get_timer() - t) / 1000} ms");
    
    t = get_timer();
    repeat 1_000_000 {
        smf_dq_normalize_opt(dq);
    }
    show_debug_message($"optimized dq normalize: {(get_timer() - t) / 1000} ms");
    
    var a = array_create(8);
    var b = array_create(8);
    var c = array_create(8);
    var d = array_create(8);
    
    t = get_timer();
    repeat 1_000_000 {
        smf_dq_quadratic_interpolate(a, b, c, 0.5, d);
    }
    show_debug_message($"base dq quadratic interpolate: {(get_timer() - t) / 1000} ms");
    
    t = get_timer();
    repeat 1_000_000 {
        smf_dq_quadratic_interpolate_opt(a, b, c, 0.5, d);
    }
    show_debug_message($"optimized dq quadratic interpolate: {(get_timer() - t) / 1000} ms");
    
    t = get_timer();
    repeat 1_000_000 {
        smf_dq_quadratic_interpolate(a, b, c, 0.5, d);
    }
    show_debug_message($"base dq set translation: {(get_timer() - t) / 1000} ms");
    
    t = get_timer();
    repeat 1_000_000 {
        smf_dq_quadratic_interpolate_opt(a, b, c, 0.5, d);
    }
    show_debug_message($"optimized dq set translation: {(get_timer() - t) / 1000} ms");
    #endregion
    
    #region matrix stuff
    var m = matrix_build_lookat(0, 0, 100, 0, 0, 0, 0, 0, 1);
    
    var t = get_timer();
    repeat 1_000_000 {
        smf_mat_invert(m);
    }
    show_debug_message($"base smf matrix invert: {(get_timer() - t) / 1000} ms");
    
    t = get_timer();
    repeat 1_000_000 {
        smf_mat_invert_fast(m);
    }
    show_debug_message($"fast smf matrix invert: {(get_timer() - t) / 1000} ms");
    
    t = get_timer();
    repeat 1_000_000 {
        matrix_inverse(m);
    }
    show_debug_message($"runtime matrix invert: {(get_timer() - t) / 1000} ms");
    #endregion
}