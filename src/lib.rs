#[cxx::bridge(namespace = "trajoptlibrust")]
mod ffi {
    #[derive(Debug, Deserialize, Serialize)]
    struct SwerveModule {
        x: f64,
        y: f64,
        wheel_radius: f64,
        wheel_max_angular_velocity: f64,
        wheel_max_torque: f64,
    }

    #[derive(Debug, Deserialize, Serialize)]
    struct SwerveDrivetrain {
        mass: f64,
        moi: f64,
        modules: Vec<SwerveModule>,
    }

    #[derive(Debug, Deserialize, Serialize)]
    struct Pose2d {
        x: f64,
        y: f64,
        heading: f64,
    }

    #[derive(Debug, Deserialize, Serialize, Clone)]
    struct HolonomicTrajectorySample {
        timestamp: f64,
        x: f64,
        y: f64,
        heading: f64,
        velocity_x: f64,
        velocity_y: f64,
        angular_velocity: f64,
        module_forces_x: Vec<f64>,
        module_forces_y: Vec<f64>,
    }

    #[derive(Debug, Deserialize, Serialize, Clone)]
    struct HolonomicTrajectory {
        samples: Vec<HolonomicTrajectorySample>,
    }

    unsafe extern "C++" {
        include!("trajoptlibrust.hpp");

        type SwervePathBuilderImpl;

        fn set_drivetrain(self: Pin<&mut SwervePathBuilderImpl>, drivetrain: &SwerveDrivetrain);
        fn set_bumpers(self: Pin<&mut SwervePathBuilderImpl>, length: f64, width: f64);
        fn set_control_interval_counts(self: Pin<&mut SwervePathBuilderImpl>, counts: Vec<usize>);

        fn pose_wpt(
            self: Pin<&mut SwervePathBuilderImpl>,
            index: usize,
            x: f64,
            y: f64,
            heading: f64,
        );
        fn translation_wpt(
            self: Pin<&mut SwervePathBuilderImpl>,
            index: usize,
            x: f64,
            y: f64,
            heading_guess: f64,
        );
        fn empty_wpt(
            self: Pin<&mut SwervePathBuilderImpl>,
            index: usize,
            x_guess: f64,
            y_guess: f64,
            heading_guess: f64,
        );

        fn sgmt_initial_guess_points(
            self: Pin<&mut SwervePathBuilderImpl>,
            from_index: usize,
            guess_points: &Vec<Pose2d>,
        );

        fn wpt_linear_velocity_direction(
            self: Pin<&mut SwervePathBuilderImpl>,
            index: usize,
            angle: f64,
        );
        fn wpt_linear_velocity_max_magnitude(
            self: Pin<&mut SwervePathBuilderImpl>,
            index: usize,
            magnitude: f64,
        );
        fn wpt_angular_velocity_max_magnitude(
            self: Pin<&mut SwervePathBuilderImpl>,
            index: usize,
            angular_velocity: f64,
        );
        fn wpt_linear_acceleration_max_magnitude(
            self: Pin<&mut SwervePathBuilderImpl>,
            index: usize,
            magnitude: f64,
        );
        fn wpt_point_at(
            self: Pin<&mut SwervePathBuilderImpl>,
            index: usize,
            field_point_x: f64,
            field_point_y: f64,
            heading_tolerance: f64,
        );

        fn sgmt_linear_velocity_direction(
            self: Pin<&mut SwervePathBuilderImpl>,
            from_index: usize,
            to_index: usize,
            angle: f64,
        );
        fn sgmt_linear_velocity_max_magnitude(
            self: Pin<&mut SwervePathBuilderImpl>,
            from_index: usize,
            to_index: usize,
            magnitude: f64,
        );
        fn sgmt_angular_velocity_max_magnitude(
            self: Pin<&mut SwervePathBuilderImpl>,
            from_index: usize,
            to_index: usize,
            angular_velocity: f64,
        );
        fn sgmt_linear_acceleration_max_magnitude(
            self: Pin<&mut SwervePathBuilderImpl>,
            from_index: usize,
            to_index: usize,
            magnitude: f64,
        );
        fn sgmt_point_at(
            self: Pin<&mut SwervePathBuilderImpl>,
            from_index: usize,
            to_index: usize,
            field_point_x: f64,
            field_point_y: f64,
            heading_tolerance: f64,
        );

        fn sgmt_circle_obstacle(
            self: Pin<&mut SwervePathBuilderImpl>,
            from_index: usize,
            to_index: usize,
            x: f64,
            y: f64,
            radius: f64,
        );

        fn sgmt_polygon_obstacle(
            self: Pin<&mut SwervePathBuilderImpl>,
            from_index: usize,
            to_index: usize,
            x: Vec<f64>,
            y: Vec<f64>,
            radius: f64,
        );

        fn generate(
            self: &SwervePathBuilderImpl,
            diagnostics: bool,
            uuid: i64,
        ) -> Result<HolonomicTrajectory>;

        fn add_progress_callback(
            self: Pin<&mut SwervePathBuilderImpl>,
            callback: fn(HolonomicTrajectory, i64),
        );

        fn cancel_all(self: Pin<&mut SwervePathBuilderImpl>);

        fn new_swerve_path_builder_impl() -> UniquePtr<SwervePathBuilderImpl>;

        fn calculate_linear_initial_guess(self: &SwervePathBuilderImpl) -> HolonomicTrajectory;

        fn calculate_spline_initial_guess(self: &SwervePathBuilderImpl) -> HolonomicTrajectory;

        fn calculate_control_interval_counts(self: &SwervePathBuilderImpl) -> Vec<usize>;
    }
}

pub struct SwervePathBuilder {
    path: cxx::UniquePtr<crate::ffi::SwervePathBuilderImpl>,
}

impl SwervePathBuilder {
    pub fn new() -> SwervePathBuilder {
        SwervePathBuilder {
            path: crate::ffi::new_swerve_path_builder_impl(),
        }
    }

    pub fn set_drivetrain(&mut self, drivetrain: &crate::ffi::SwerveDrivetrain) {
        crate::ffi::SwervePathBuilderImpl::set_drivetrain(self.path.pin_mut(), drivetrain);
    }

    pub fn set_bumpers(&mut self, length: f64, width: f64) {
        crate::ffi::SwervePathBuilderImpl::set_bumpers(self.path.pin_mut(), length, width);
    }

    pub fn set_control_interval_counts(&mut self, counts: Vec<usize>) {
        crate::ffi::SwervePathBuilderImpl::set_control_interval_counts(self.path.pin_mut(), counts);
    }

    pub fn pose_wpt(&mut self, index: usize, x: f64, y: f64, heading: f64) {
        crate::ffi::SwervePathBuilderImpl::pose_wpt(self.path.pin_mut(), index, x, y, heading);
    }

    pub fn translation_wpt(&mut self, index: usize, x: f64, y: f64, heading_guess: f64) {
        crate::ffi::SwervePathBuilderImpl::translation_wpt(
            self.path.pin_mut(),
            index,
            x,
            y,
            heading_guess,
        );
    }

    pub fn empty_wpt(&mut self, index: usize, x_guess: f64, y_guess: f64, heading_guess: f64) {
        crate::ffi::SwervePathBuilderImpl::empty_wpt(
            self.path.pin_mut(),
            index,
            x_guess,
            y_guess,
            heading_guess,
        );
    }

    pub fn sgmt_initial_guess_points(
        &mut self,
        from_index: usize,
        guess_points: &Vec<crate::ffi::Pose2d>,
    ) {
        crate::ffi::SwervePathBuilderImpl::sgmt_initial_guess_points(
            self.path.pin_mut(),
            from_index,
            guess_points,
        );
    }

    pub fn wpt_linear_velocity_direction(&mut self, index: usize, angle: f64) {
        crate::ffi::SwervePathBuilderImpl::wpt_linear_velocity_direction(
            self.path.pin_mut(),
            index,
            angle,
        );
    }

    pub fn wpt_linear_velocity_max_magnitude(&mut self, index: usize, magnitude: f64) {
        crate::ffi::SwervePathBuilderImpl::wpt_linear_velocity_max_magnitude(
            self.path.pin_mut(),
            index,
            magnitude,
        );
    }

    pub fn wpt_angular_velocity_max_magnitude(&mut self, index: usize, angular_velocity: f64) {
        crate::ffi::SwervePathBuilderImpl::wpt_angular_velocity_max_magnitude(
            self.path.pin_mut(),
            index,
            angular_velocity,
        );
    }

    pub fn wpt_linear_acceleration_max_magnitude(&mut self, index: usize, magnitude: f64) {
        crate::ffi::SwervePathBuilderImpl::wpt_linear_acceleration_max_magnitude(
            self.path.pin_mut(),
            index,
            magnitude,
        );
    }

    pub fn wpt_point_at(
        &mut self,
        index: usize,
        field_point_x: f64,
        field_point_y: f64,
        heading_tolerance: f64,
    ) {
        crate::ffi::SwervePathBuilderImpl::wpt_point_at(
            self.path.pin_mut(),
            index,
            field_point_x,
            field_point_y,
            heading_tolerance,
        )
    }

    pub fn sgmt_linear_velocity_direction(
        &mut self,
        from_index: usize,
        to_index: usize,
        angle: f64,
    ) {
        crate::ffi::SwervePathBuilderImpl::sgmt_linear_velocity_direction(
            self.path.pin_mut(),
            from_index,
            to_index,
            angle,
        );
    }

    pub fn sgmt_linear_velocity_max_magnitude(
        &mut self,
        from_index: usize,
        to_index: usize,
        magnitude: f64,
    ) {
        crate::ffi::SwervePathBuilderImpl::sgmt_linear_velocity_max_magnitude(
            self.path.pin_mut(),
            from_index,
            to_index,
            magnitude,
        );
    }

    pub fn sgmt_angular_velocity_max_magnitude(
        &mut self,
        from_index: usize,
        to_index: usize,
        angular_velocity: f64,
    ) {
        crate::ffi::SwervePathBuilderImpl::sgmt_angular_velocity_max_magnitude(
            self.path.pin_mut(),
            from_index,
            to_index,
            angular_velocity,
        );
    }

    pub fn sgmt_linear_acceleration_max_magnitude(
        &mut self,
        from_index: usize,
        to_index: usize,
        magnitude: f64,
    ) {
        crate::ffi::SwervePathBuilderImpl::sgmt_linear_acceleration_max_magnitude(
            self.path.pin_mut(),
            from_index,
            to_index,
            magnitude,
        );
    }

    pub fn sgmt_point_at(
        &mut self,
        from_index: usize,
        to_index: usize,
        field_point_x: f64,
        field_point_y: f64,
        heading_tolerance: f64,
    ) {
        crate::ffi::SwervePathBuilderImpl::sgmt_point_at(
            self.path.pin_mut(),
            from_index,
            to_index,
            field_point_x,
            field_point_y,
            heading_tolerance,
        )
    }

    pub fn sgmt_circle_obstacle(
        &mut self,
        from_index: usize,
        to_index: usize,
        x: f64,
        y: f64,
        radius: f64,
    ) {
        crate::ffi::SwervePathBuilderImpl::sgmt_circle_obstacle(
            self.path.pin_mut(),
            from_index,
            to_index,
            x,
            y,
            radius,
        );
    }

    pub fn sgmt_polygon_obstacle(
        &mut self,
        from_index: usize,
        to_index: usize,
        x: Vec<f64>,
        y: Vec<f64>,
        radius: f64,
    ) {
        crate::ffi::SwervePathBuilderImpl::sgmt_polygon_obstacle(
            self.path.pin_mut(),
            from_index,
            to_index,
            x,
            y,
            radius,
        );
    }

    ///
    /// Generate the trajectory;
    ///
    /// * diagnostics: If true, prints per-iteration details of the solver to stdout.
    /// * handle: A number used to identify results from this generation in the
    /// `add_progress_callback` callback. If `add_progress_callback` has not been called, this
    /// value has no significance.
    ///
    /// Returns a result with either the final `trajoptlib::HolonomicTrajectory`, or a String error message
    /// if generation failed.
    ///
    pub fn generate(
        &mut self,
        diagnostics: bool,
        handle: i64,
    ) -> Result<HolonomicTrajectory, String> {
        match self.path.generate(diagnostics, handle) {
            Ok(traj) => Ok(traj),
            Err(msg) => Err(msg.what().to_string()),
        }
    }

    ///
    /// Add a callback that will be called on each iteration of the solver.
    ///
    /// * callback: a `fn` (not a closure) to be executed. The callback's
    /// first parameter will be a `trajoptlib::HolonomicTrajectory`, and the second
    /// parameter will be an `i64` equal to the handle passed in `generate()`
    ///
    /// This function can be called multiple times to add multiple callbacks.
    ///
    pub fn add_progress_callback(&mut self, callback: fn(HolonomicTrajectory, i64)) {
        crate::ffi::SwervePathBuilderImpl::add_progress_callback(self.path.pin_mut(), callback);
    }

    pub fn cancel_all(&mut self) {
        crate::ffi::SwervePathBuilderImpl::cancel_all(self.path.pin_mut());
    }

    pub fn calculate_linear_initial_guess(&self) -> HolonomicTrajectory {
        self.path.calculate_linear_initial_guess()
    }

    pub fn calculate_spline_initial_guess(&self) -> HolonomicTrajectory {
        self.path.calculate_spline_initial_guess()
    }

    pub fn calculate_control_interval_counts(&self) -> Vec<usize> {
        self.path.calculate_control_interval_counts()
    }
}

impl Default for SwervePathBuilder {
    fn default() -> Self {
        Self::new()
    }
}

pub use ffi::HolonomicTrajectory;
pub use ffi::HolonomicTrajectorySample;
pub use ffi::Pose2d;
pub use ffi::SwerveDrivetrain;
pub use ffi::SwerveModule;
