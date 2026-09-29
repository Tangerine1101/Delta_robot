Calc_inverse_kinematics:
'
// Khởi tạo lại cờ báo lỗi ở mỗi đầu chu kỳ quét (Primary Task)
Out_Error_IK := FALSE;

// -----------------------------------------------------------------------
// TÍNH TOÁN THETA 1 (Cánh tay 1 - Không xoay trục)
// -----------------------------------------------------------------------
Calc_Inverse_Kinematics := Calc_Angles_YZ(
    X0 := X, 
    Y0 := Y, 
    Z0 := Z,
    OutTheta => tmp_Theta1
);

	
// [TỐI ƯU B] Xử lý lỗi an toàn: Nếu tính toán IK thất bại (Tọa độ ngoài tầm với)
IF Calc_Inverse_Kinematics = TRUE  OR tmp_Theta1 <-20 THEN
    Out_Error_IK := TRUE; // Bật cờ báo lỗi ra hệ thống
    RETURN;               // Ngắt tính toán ngay lập tức
END_IF;

// [TỐI ƯU A] Truyền thẳng giá trị LREAL nguyên bản xuống Servo, BỎ HÀM ROUND
Theta1 := tmp_Theta1;


// -----------------------------------------------------------------------
// TÍNH TOÁN THETA 2 (Cánh tay 2 - Xoay hệ tọa độ đi -120 độ)
// -----------------------------------------------------------------------
rotate_X2 := X * Cos120 + Y * Sin120;
rotate_Y2 := Y * Cos120 - X * Sin120;

Calc_Inverse_Kinematics := Calc_Angles_YZ(
    X0 := rotate_X2, 
    Y0 := rotate_Y2, 
    Z0 := Z,
    OutTheta => tmp_Theta2
);

IF Calc_Inverse_Kinematics = TRUE OR tmp_Theta2 <-20 THEN
    Out_Error_IK := TRUE;
    RETURN;
END_IF;

Theta2 := tmp_Theta2;


// -----------------------------------------------------------------------
// TÍNH TOÁN THETA 3 (Cánh tay 3 - Xoay hệ tọa độ đi +120 độ)
// -----------------------------------------------------------------------
rotate_X3 := X * Cos120 - Y * Sin120;
rotate_Y3 := Y * Cos120 + X * Sin120;

Calc_Inverse_Kinematics := Calc_Angles_YZ(
    X0 := rotate_X3, 
    Y0 := rotate_Y3, 
    Z0 := Z,
    OutTheta => tmp_Theta3
);

IF Calc_Inverse_Kinematics = TRUE OR tmp_Theta3*-1 >20 THEN
    Out_Error_IK := TRUE;
    RETURN;
END_IF;

Theta3 := tmp_Theta3*-1;'

Calc_Angles_YZ:
'(*
// FUNCTION: Calc_Angles_YZ (Cal the inverse in 2D-space)
//All variants use Lreal to uttilize the precision of servo and syns all data

// [] Bẫy lỗi chia cho 0: Nếu tọa độ Z0 = 0 (End-effector nằm ngang với Base)
// Phép tính 'a' và 'b' bên dưới sẽ bị lỗi chia cho 0 làm CPU PLC báo lỗi (Major Fault)
*)
IF Z0 = 0.0 THEN
    Calc_Angles_YZ := TRUE; // turn Error true
    RETURN;
END_IF;

// Calc the Y1 and have temp_y0 (the prosition of the Y0 with Y1)
y1 := -0.5 * Tan30* Base;   // the point of edge of fix-triangle
tmp_y0 := Y0 - (0.5 *Tan30* EndEffector);  // the point of the edge of active-triagle

// we have ellbow (0,yj,zj) is contraint with:
//1. on a circle (0,y1,0) with R = bicep so (yj-y1)^2 + z_j^2 = bicep^2
//2. distance from End-effector (X0, tmp_y0,z0) equal the Forearm so
// X0^2 + (tmp_y0 - yj)^2 + (z-zj)^2 = forearm^2
// mathemitical tranformation we have linear function z with a and b bellow
// zj = a + b*yj => we have zj and we tranfer it to equation 1 and solve the two level function with yj
a := (X0*X0 + tmp_y0 * tmp_y0 + Z0 * Z0 + Bicep * Bicep - Forearm * Forearm - y1 * y1) / (2.0 * Z0);
b := (y1 - tmp_y0) / Z0;

// the delta of two level function with yj
d := - (a + b * y1) * (a + b * y1) + Bicep * (b * b * Bicep + Bicep);

// Kiểm tra xem điểm có tồn tại trong vùng không gian (Workspace) không
IF d < 0.0 THEN
    Calc_Angles_YZ := TRUE; // Trả về lỗi (Điểm không với tới được)
    RETURN;
END_IF;

yj := (y1 - a * b - SQRT(d)) / (b * b + 1.0);
zj := a + b * yj;

tmp_AnglePi := 0.0;
IF yj > y1 THEN
    tmp_AnglePi := 180.0;
END_IF;

// [TỐI ƯU 3] Bẫy lỗi chia cho 0 trong hàm tính góc ATAN
// Khi y1 = yj, phép chia (-zj / (y1 - yj)) sẽ gây lỗi PLC
IF (y1 - yj) = 0.0 THEN
    IF -zj >= 0.0 THEN
        OutTheta := 90.0 + tmp_AnglePi;
    ELSE
        OutTheta := -90.0 + tmp_AnglePi;
    END_IF;
ELSE
    // Tính góc Theta. Sử dụng hằng số '_pi' có sẵn của Sysmac Studio
    OutTheta := 180.0 * ATAN(-zj / (y1 - yj)) / Pi + tmp_AnglePi;
END_IF;

// Trả về FALSE khi tính toán thành công
Calc_Angles_YZ := FALSE;
RETURN;'

Calc_forward_kinematics:
'// =======================================================================
// FUNCTION: Calc_Forward_Kinematics (Tính toán động học thuận Robot Delta 3D)
// Trả về FALSE nếu tính toán thành công, TRUE nếu điểm nằm ngoài vùng làm việc
// =======================================================================
// LẤY GIÁ TRỊ GÓC THỰC TẾ (ACTUAL POSITION) TỪ ENCODER CỦA TRỤC
// =======================================================================
// Chuyển đổi các góc hiện tại từ Độ sang Radian
t1 := Theta1 * Pi / 180.0;
t2 := Theta2 * Pi / 180.0;
t3 := Theta3 * Pi / -180.0;

// Tính toán độ lệch tâm tương đối giữa Base và EndEffector
// (w chính là dịch tâm ngang từ gốc tọa độ đến khớp khuỷu tay nếu chiếu lên mặt phẳng XY)
w := 0.5 * Tan30 * (Base - EndEffector);

// =======================================================================
// BƯỚC 1: Tìm tọa độ không gian (x, y, z) của 3 khớp khuỷu tay (Elbow Joints)
// Hệ tọa độ tiêu chuẩn: Trục 1 nằm dọc theo phần âm của trục Y
// =======================================================================

// Trục 1 (Góc lệch 0 độ so với trục tính toán, xoay về -Y)
x_j1 := 0.0;
y_j1 := -(w + Bicep * COS(t1));
z_j1 := -Bicep * SIN(t1);

// Trục 2 (Xoay +120 độ)
x_j2 := (w + Bicep * COS(t2)) * Cos30;
y_j2 := (w + Bicep * COS(t2)) * Sin30;
z_j2 := -Bicep * SIN(t2);

// Trục 3 (Xoay -120 độ hoặc 240 độ)
x_j3 := -(w + Bicep * COS(t3)) * Cos30;
y_j3 := (w + Bicep * COS(t3)) * Sin30;
z_j3 := -Bicep * SIN(t3);

// =======================================================================
// BƯỚC 2: Giải hệ 3 phương trình mặt cầu có tâm tại các Elbow, bán kính Forearm
// =======================================================================

// =======================================================================
// BƯỚC 2: Giải hệ 3 phương trình mặt cầu (Dạng tổng quát, KHÔNG RÚT GỌN SAI)
// =======================================================================
// BƯỚC 2: Giải hệ 3 phương trình mặt cầu có tâm tại các Elbow (Phiên bản Tổng quát)
// =======================================================================

// Tính tổng bình phương tọa độ của các Elbow
r1 := y_j1 * y_j1 + z_j1 * z_j1;  // x_j1 = 0
r2 := x_j2 * x_j2 + y_j2 * y_j2 + z_j2 * z_j2;
r3 := x_j3 * x_j3 + y_j3 * y_j3 + z_j3 * z_j3;

// Thiết lập các hệ số cho hệ phương trình tuyến tính 2 ẩn (X, Y) theo tham số Z
// Dạng: 
// A1*X + B1*Y = D1 - C1*Z
// A2*X + B2*Y = D2 - C2*Z

A1_coeff := x_j2;                   // Thực chất là (x_j2 - x_j1) nhưng x_j1 = 0
B1_coeff := y_j2 - y_j1;
C1_coeff := z_j2 - z_j1;
D1_coeff := 0.5 * (r2 - r1);

A2_coeff := x_j3;                   // Thực chất là (x_j3 - x_j1) nhưng x_j1 = 0
B2_coeff := y_j3 - y_j1;
C2_coeff := z_j3 - z_j1;
D2_coeff := 0.5 * (r3 - r1);

// Tính định thức (Determinant) của hệ phương trình
det := (A1_coeff * B2_coeff) - (A2_coeff * B1_coeff);

// Bẫy lỗi: Hệ vô nghiệm (Chỉ xảy ra khi 3 khuỷu tay cùng nằm trên 1 đường thẳng)
IF det = 0.0 THEN
    Calc_Forward_Kinematics := TRUE; 
    RETURN;
END_IF;

// Sử dụng quy tắc Cramer để tìm a1, b1 (Cho X) và a2, b2 (Cho Y)
// Để biểu diễn: X0 = a1 + b1*Z0
a1 := (D1_coeff * B2_coeff - D2_coeff * B1_coeff) / det;
b1 := -(C1_coeff * B2_coeff - C2_coeff * B1_coeff) / det;

// Để biểu diễn: Y0 = a2 + b2*Z0
a2 := (A1_coeff * D2_coeff - A2_coeff * D1_coeff) / det;
b2 := -(A1_coeff * C2_coeff - A2_coeff * C1_coeff) / det;

// =======================================================================
// BƯỚC 3: Thay X0, Y0 vào phương trình mặt cầu 1 để tìm Z0
// Tạo thành phương trình bậc 2: A_quad * Z0^2 + B_quad * Z0 + C_quad = 0
// =======================================================================

A_quad := b1 * b1 + b2 * b2 + 1.0;
B_quad := 2.0 * (a1 * b1 + a2 * b2 - b2 * y_j1 - z_j1);
C_quad := a1 * a1 + a2 * a2 - 2.0 * a2 * y_j1 + r1 - (Forearm * Forearm);

// Tính Delta (Discriminant)
d := (B_quad * B_quad) - (4.0 * A_quad * C_quad);

// Kiểm tra xem vị trí các góc hiện tại có tạo ra tọa độ hợp lệ không
IF d < 0.0 THEN
    Calc_Forward_Kinematics := TRUE; // Trả về lỗi: Các khớp không thể kết nối
    RETURN;
END_IF;

// Tính tọa độ trung tâm End-Effector (Z lấy nghiệm có dấu trừ phía trước SQRT 
// vì đặc thù cơ cấu Delta robot làm việc ở nửa dưới trục Z)
OutZ := (-B_quad - SQRT(d)) / (2.0 * A_quad);

// Đẩy ngược Z0 vào hệ số tuyến tính để tìm X0 và Y0
OutX := a1 + b1 * OutZ;
OutY := a2 + b2 * OutZ;

Calc_OutZ := LREAL_TO_REAL(OutZ);
Calc_OutX := LREAL_TO_REAL(a1 + b1 * OutZ);
Calc_OutY := LREAL_TO_REAL(a2 + b2 * OutZ);

// Tính toán thành công
Calc_Forward_Kinematics := FALSE;
RETURN;'
