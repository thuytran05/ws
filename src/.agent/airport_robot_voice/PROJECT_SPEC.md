# Hoàn thiện airport_robot_voice trên ROS 2 Jazzy

Tài liệu này là yêu cầu triển khai cho Antigravity. Đọc toàn bộ, khảo sát code hiện tại rồi thực hiện các giai đoạn ở cuối tài liệu theo thứ tự. Hoàn thiện code và kiểm thử thực tế trong container đang có. Cập nhật tiến độ sau từng giai đoạn và tiếp tục các phần có thể thực hiện khi một phần gặp trở ngại.

## 1. Bối cảnh và kết quả cần đạt

- Project Antigravity đang mở tại thư mục `src` của ROS workspace. Đường dẫn workspace đã từng là `/home/ws`, cần xác minh trong phiên hiện tại.
- Máy ThinkPad P15 Gen 2 chạy Ubuntu 22.04. ROS 2 Jazzy chạy trong Docker/Dev Container. Host có ROS 2 Humble. Thực thi build, node và kiểm thử ROS trong container Jazzy.
- Package `airport_robot_voice` đã được copy vào `src` và chạy thử bằng bàn phím. Micro và loa chưa được xác minh trên container này.
- Tham khảo lịch sử tại `https://github.com/thuytran05/Mobile-robot`. Code local và thay đổi hiện có của người dùng là nguồn triển khai chính.
- Các package khác gồm Linorobot2, camera và nhận diện người đang cùng workspace. Công việc hiện tại tập trung vào giọng nói và giao tiếp với navigation.
- Hệ thống dùng tiếng Việt, xử lý offline. Dùng tiếp Vosk cho STT, Piper hoặc eSpeak cho TTS. Cho phép tải model và dependency khi thiết lập, sau đó vận hành được khi ngắt Internet.
- Đầu vào thử nghiệm gồm bàn phím và micro. Bổ sung đọc WAV để kiểm tra STT lặp lại bằng cùng một bản thu.
- Đầu ra gồm câu trả lời thông tin và yêu cầu điều hướng có cấu trúc. Có node riêng để gửi goal Nav2, quản lý nhiệm vụ và phản hồi kết quả.

Hai cách vận hành phải có sẵn.

| Chế độ | Hành vi |
| --- | --- |
| Voice độc lập | Nhận câu, hiểu ý định, trả lời thông tin hoặc xuất yêu cầu điểm đến. Các node voice hoạt động khi Nav2 chưa chạy. |
| Voice kết hợp navigation | Node navigation nhận yêu cầu từ voice, chọn đích hợp lệ, gửi action Nav2, quản lý tạm dừng, tiếp tục, hủy, đổi đích và phát kết quả. |

Ưu tiên chạy từng node bằng `ros2 run`. Tận dụng launch hiện có nếu phù hợp, luôn cho phép chọn bật hoặc tắt node navigation.

## 2. Quy định về file và thư mục

### 2.1. Một nơi chung cho toàn bộ tài liệu phục vụ AI

Chọn `<src>/.agent/` làm thư mục AI dùng chung cho project. Tên có dấu chấm được chọn để phù hợp cơ chế discovery của Antigravity. Xác minh cơ chế này với phiên bản Antigravity đang chạy. Nếu có sẵn cấu hình AI của người dùng, đọc và tích hợp có chọn lọc.

Mọi file do agent chủ động tạo để phục vụ việc làm việc của AI phải nằm trong thư mục này, bao gồm rule, skill, prompt, kế hoạch, checklist, báo cáo kiểm thử, ghi chú điều tra lỗi và tài liệu tiếp tục công việc. Áp dụng cùng quy tắc này khi làm package khác trong project.

Các đường dẫn cụ thể được chọn cho dự án này.

| Mục đích | Đường dẫn tính từ `src` |
| --- | --- |
| Đặc tả này | `.agent/airport_robot_voice/PROJECT_SPEC.md` |
| Tiến độ và kết quả kiểm thử | `.agent/airport_robot_voice/PROGRESS.md` |
| Rule dùng chung | `.agent/rules/workspace.md` |
| Skill thực sự cần thêm | `.agent/skills/<skill-name>/SKILL.md` |
| Chứng cứ bổ sung thực sự cần giữ | `.agent/airport_robot_voice/evidence/` |

Giữ kế hoạch và kết quả trong một file `PROGRESS.md` để dễ theo dõi. Chỉ tạo thêm tài liệu AI khi cần giữ thông tin mà file này không chứa hợp lý được. Khi bắt đầu cuộc hội thoại mới, đọc rule, đặc tả và tiến độ trước khi sửa code.

Rule luôn hoạt động phải ngắn, ghi rõ giới hạn thư mục và đường dẫn đặc tả cần đọc. Kiểm tra format rule đúng phiên bản Antigravity. Tài liệu hiện hành mô tả rule `.md` với frontmatter `trigger: always_on`, cần đối chiếu thực tế. Nếu discovery chưa hoạt động, tiếp tục đọc rule và skill bằng đường dẫn rõ ràng, ghi lại tình trạng để xử lý.

### 2.2. Các file của phần mềm ROS

Được tạo và sửa file thực thi, module Python, cấu hình, test và metadata package ở đúng vị trí thông thường trong `airport_robot_voice`. File phục vụ ROS được phân loại theo vai trò, dù AI là bên viết file đó.

- Module và node Python nằm trong thư mục Python package.
- Cấu hình chạy nằm trong `config`.
- Launch nằm trong `launch` nếu cần.
- Test có thể chạy lại nằm trong `test` hoặc `tests`, theo quy ước đã có.
- `setup.py`, `setup.cfg`, `package.xml` và README sử dụng package được sửa khi cần để cài và chạy đúng.
- Tận dụng cấu trúc hiện tại, tránh di chuyển toàn bộ package chỉ để đổi cách tổ chức.
- Việc sửa package khác phải có phụ thuộc kỹ thuật cụ thể và được ghi lý do trong tiến độ.
- File build, install, log của colcon tiếp tục theo cấu trúc ROS workspace. Dữ liệu thu âm và log vận hành có thư mục runtime cấu hình được, tách khỏi source. Tài liệu AI vẫn tập trung trong `.agent`.
- Theo dõi trạng thái Git trước và sau công việc, bảo toàn thay đổi người dùng, kể cả file untracked. Thực hiện commit hoặc push khi người dùng yêu cầu.
- Dọn các process thử nghiệm do chính phiên này tạo sau khi test. Giữ các node, Gazebo và phiên ROS do người dùng đang chạy.

## 3. Skill và tài liệu kỹ thuật

Khảo sát các skill thực sự có sẵn và chỉ sử dụng phần phù hợp. Được cài thêm skill cần thiết cho nhiệm vụ này sau khi xem nội dung, nguồn và phạm vi tác động. Ghi lại tên thật, nguồn, phiên bản hoặc commit nếu có và mục đích trong `PROGRESS.md`.

Ba nhóm năng lực cần dùng là phát triển package ROS 2 Jazzy bằng Python, quản lý vòng đời action Nav2, và kiểm thử giọng nói offline trong Docker. Đây là nhóm công việc cần hỗ trợ. Tên skill có sẵn phải được xác minh qua discovery hoặc nguồn chính thức.

Ưu tiên dùng skill đã có. Khi thiếu hướng dẫn có thể tái sử dụng, được viết một skill ngắn trong `.agent/skills/`, bám đúng workflow của dự án này. Cài đặt ở phạm vi project. Tiếp tục triển khai khi thiếu skill bằng cách đọc tài liệu chính thức.

Tra API theo đúng ROS 2 Jazzy và phiên bản Vosk/Piper/eSpeak thực tế. Kiểm tra bằng runtime và code đã cài. Tài liệu ngoài là tài liệu tham khảo kỹ thuật. Quy định dự án nằm trong yêu cầu người dùng và đặc tả này.

Tài liệu Antigravity tham khảo khi thiết lập discovery và skill.

- https://antigravity.google/docs/skills/
- https://antigravity.google/docs/rules/
- https://antigravity.google/docs/customization-discovery/

## 4. Khảo sát code trước khi sửa

Đọc toàn bộ `airport_robot_voice`, đặc biệt là `nlu.py`, `stt_node.py`, `intent_node.py`, `tts_node.py`, `nav_command_node.py`, hai file JSON trong `config`, launch, script test và metadata package.

Xác minh phiên bản hiện có của những vấn đề đã được phát hiện ở bản GitHub trước đây.

1. Hỏi vị trí như “cổng A3 ở đâu” có `action = null` nên chưa dẫn đường.
2. Danh mục mẫu có A1, A2, B1, B2 và 15, chưa có A3.
3. NLU có thể tạo `target` của mã cổng chưa tồn tại và thông báo sẽ dẫn đi trước khi node navigation báo lỗi.
4. Thiếu đích thì hệ thống hỏi tiếp. Yêu cầu hiện tại là thông báo ngắn rồi kết thúc lượt.
5. Phủ định như “đừng dẫn tôi đến cổng A1” có thể vẫn sinh lệnh navigate.
6. Regex có thể đọc A123 thành A12. Alias ngắn có thể khớp bên trong chuỗi dài.
7. Danh mục chỉ có một điểm toilet, chưa có chọn đường đi tới nhiều điểm cùng loại.
8. Callback navigation dùng chung target name và goal handle, dễ lẫn trạng thái khi có yêu cầu mới.
9. Cancel chưa theo dõi đầy đủ khi goal đang chờ chấp nhận hoặc hủy chưa hoàn tất.
10. Đường dẫn config mặc định suy ra từ `__file__` có thể không đúng sau cài package.
11. STT có thể giữ wake state mãi sau lần gọi đầu, hàng đợi audio chưa được kiểm soát đầy đủ.
12. Launch keyboard cần xác minh khả năng nhận stdin. Luồng terminal riêng phải hoạt động được.
13. Bộ test từng đạt 57/57 chủ yếu kiểm tra tên intent. Kết quả này chưa xác minh target, action, micro, loa và Nav2.

Ghi kết quả khảo sát bằng file và hàm cụ thể. Sửa theo code local thực tế.

## 5. Yêu cầu chức năng

### 5.1. Hiểu câu và chốt điểm đến

| Câu hoặc tình huống | Kết quả yêu cầu |
| --- | --- |
| “Dẫn tôi đến cổng A3” | Xác định A3 trong danh mục. Khi có điểm hợp lệ và navigation sẵn sàng, thông báo rồi bắt đầu điều hướng. |
| “Cổng A3 ở đâu?” | Cùng nhánh dẫn đường đến A3. |
| “Cho tôi tới quầy check-in” | Nhận tên gọi khác và chọn điểm tương ứng. |
| “Dẫn tôi đến nhà vệ sinh”, “nhà vệ sinh ở đâu” | Giải quyết nhóm restroom và chọn điểm theo đường đi khả dụng. |
| “Dẫn tôi đến…” | Báo chưa xác định được điểm đến, kết thúc lượt. |
| Mã cổng hoặc địa điểm chưa có | Báo địa điểm chưa có trong danh mục, kết thúc lượt. |
| Có nhiều đích cụ thể trong cùng một câu, như “A1 hay A2” | Báo yêu cầu chưa rõ và kết thúc lượt. |
| “Đừng dẫn tôi đến cổng A1” | Nhận phủ định, giữ yêu cầu này không phát sinh chuyến mới. |
| “Không cần nữa”, “hủy chuyến” | Nhận đây là lệnh hủy nhiệm vụ. |
| Câu ngoài phạm vi | Phản hồi ngắn, kết thúc lượt. |

Hệ thống không yêu cầu xác nhận và không mở hội thoại hỏi lại để thu thập đích còn thiếu. Tắt cơ chế pending slot của bản cũ cho luồng này. Lượt nói sau được xử lý như yêu cầu mới.

Chuẩn hóa tiếng Việt có dấu, không dấu và Unicode tổ hợp. Hỗ trợ các cách đọc mã như “a ba”, “bê hai”, “cổng số mười lăm” theo bộ mã đã cấu hình. Khớp trọn token và trọn mã. Mã không tồn tại phải được phát hiện trước khi thông báo bắt đầu. Điểm khớp NLU là điểm nội bộ của thuật toán, cần được ghi rõ ý nghĩa.

Phân biệt mệnh lệnh, câu hỏi thông tin và phủ định trong ngữ cảnh. Câu “không cần nữa” vẫn hủy chuyến. Câu “không dừng lại” không được biến thành lệnh pause. Với câu không thể phân loại chắc chắn, phản hồi chưa hiểu và kết thúc lượt.

### 5.2. Trả lời thông tin offline

Hỗ trợ câu chào, cảm ơn, hỏi cách sử dụng, thông tin dịch vụ trong danh mục, ví dụ tên Wi-Fi, có điểm sạc điện thoại hay giờ mở cửa của dịch vụ nếu dữ liệu có sẵn. Câu hỏi thông tin được trả lời khi robot đang rảnh hoặc đang thực hiện chuyến.

Theo yêu cầu người dùng, câu hỏi vị trí “ở đâu/chỗ nào” của địa điểm hợp lệ chuyển sang dẫn đường. Câu “Wi-Fi tên gì”, “dịch vụ này mở cửa mấy giờ” thuộc nhánh thông tin.

Dữ liệu demo cần được đánh dấu là minh họa trong cấu hình và tài liệu sử dụng. Câu trả lời phải lấy từ dữ liệu đã có. Khi thiếu dữ liệu, báo chưa có thông tin. Chức năng tra cứu chuyến bay của cấu hình cũ được loại khỏi khả năng công bố và runtime của bản này.

Mô tả tương đối như “bên phải bạn, cách 20 mét” cần được tính đúng theo vị trí và hệ tọa độ hiện tại. Dùng mô tả địa điểm cố định khi chưa có phép tính đó.

### 5.3. Điều khiển chuyến và hỏi trạng thái

| Lệnh | Ngữ nghĩa được chọn |
| --- | --- |
| “Dừng lại”, “chờ tôi”, “tạm dừng”, “đợi một chút” | PAUSE, giữ điểm đến để có thể tiếp tục. |
| “Đi tiếp”, “tiếp tục” | RESUME tới điểm đang giữ nếu đủ điều kiện. |
| “Hủy”, “hủy chuyến”, “không cần nữa” | CANCEL, kết thúc nhiệm vụ và xóa đích đang giữ. |
| “Đổi sang cổng A4”, một yêu cầu dẫn mới hợp lệ khi đang dẫn | REPLACE, thay đích theo quy trình ở phần vòng đời nhiệm vụ. |
| “Bạn đang dẫn tôi đi đâu?”, “còn bao xa?”, “đã tới chưa?” | STATUS, trả lời theo trạng thái và feedback thực tế. |
| “Bạn làm được gì?”, “cách sử dụng” | HELP, trả lời hướng dẫn ngắn. |

Một lệnh điều khiển được xử lý theo trạng thái thật. Khi chưa có chuyến, pause/cancel phải trả lời phù hợp. Resume sau cancel hoặc sau khởi động lại không được tự khôi phục một chuyến cũ.

Các giá trị distance và ETA chỉ được trả lời khi có feedback còn phù hợp. Khi thiếu, trả lời trạng thái và tên đích đã biết.

## 6. Kiến trúc ROS và giao tiếp

Giữ các vai trò đã có để tận dụng code.

- `stt_node` nhận keyboard, micro hoặc WAV và phát transcript.
- `intent_node` dùng lõi NLU để phân tích câu, trả lời thông tin, phát yêu cầu có cấu trúc, nhận phản hồi nhiệm vụ để trả lời status.
- `tts_node` nhận nội dung cần đọc, quản lý synthesize/playback và trạng thái phát tiếng.
- `nav_command_node` nhận yêu cầu điều hướng, chọn điểm cuối, quản lý state và action Nav2. Node này có thể chạy riêng và có thể được thay bằng thành phần navigation khác qua giao tiếp đã định nghĩa.

Lõi NLU là Python thuần để kiểm thử độc lập. Tận dụng helper và dữ liệu hiện có. Tách helper khi giúp đọc và kiểm thử logic rõ hơn. Chọn cơ chế bất đồng bộ hoặc executor phù hợp để lệnh hủy vẫn được xử lý khi đang chờ planner, goal response hoặc TTS.

Giữ các topic `/voice/text`, `/voice/intent`, `/voice/say`, `/voice/speaking` tương thích ở mức hợp lý. Document mọi thay đổi về ngữ nghĩa và schema. Được thêm topic trạng thái, phản hồi và ack khi cần để thực hiện các yêu cầu. Cùng một ý định phải được gửi đi đúng một đường thực thi để tránh hai node cùng kích hoạt goal.

Với phiên bản này, ưu tiên JSON rõ schema trong `std_msgs/String` đang có. Chỉ thêm package interface khi có lợi ích kỹ thuật cụ thể. Mọi payload được validate và báo lỗi có cấu trúc.

### 6.1. Thông tin yêu cầu

Chốt và ghi schema trước khi code phần giao tiếp. Yêu cầu điều hướng cần có ít nhất các thông tin sau.

- `schema_version`.
- `request_id` duy nhất cho lượt yêu cầu, có cơ chế khử trùng lặp trong phạm vi phiên chạy.
- Văn bản gốc, intent, action và điểm khớp.
- Đích cụ thể `target_id`, hoặc nhóm `category` và chính sách chọn điểm.
- Trạng thái đã xác định một điểm hay còn chờ chọn trong một nhóm.

Phản hồi cần liên kết request, mission và goal tương ứng, gồm trạng thái, mã lý do và tên đích khi đã chốt. Các câu thông báo cho khách chỉ được phát một lần cho cùng một sự kiện.

Lưu trạng thái mới nhất để node voice khởi động sau có thể đọc được. Chọn QoS phù hợp cho trạng thái. Topic lệnh và transcript dùng ngữ nghĩa sự kiện mới, tránh phát lại lệnh cũ khi một node vừa kết nối lại. Giới hạn bộ nhớ cho các cache và hàng đợi.

### 6.2. Voice hoạt động khi thiếu navigation

Voice độc lập vẫn trích được một `target_id` hợp lệ hoặc một yêu cầu chọn trong nhóm và phát kết quả có cấu trúc. Có thể in câu “Đã xác định yêu cầu tới cổng A3” ở terminal khi kiểm thử.

Khi chưa có bộ xử lý navigation, phản hồi phải thể hiện yêu cầu đã được hiểu và navigation chưa sẵn sàng. Chỉ node tiếp nhận và thực hiện nhiệm vụ mới được phát sự kiện bắt đầu hoặc đã đến nơi.

Với nhiều nhà vệ sinh mà chưa có planner, giữ yêu cầu nhóm ở trạng thái chờ giải quyết hoặc báo chưa thể chọn theo đường đi. Thể hiện rõ sự khác nhau giữa đích đã chọn và danh sách ứng viên.

## 7. Danh mục địa điểm và giải quyết đích

Tiếp tục dùng JSON của package nếu phù hợp. Dữ liệu cần hỗ trợ id, tên hiển thị, aliases, category, enabled, pose `x/y/yaw`, frame `map`, map id nếu nhiều bản đồ, mô tả và thông tin dịch vụ liên quan. Giữ khả năng đọc dữ liệu cũ có thể chuyển đổi chính xác. Alias trùng hoặc pose không hợp lệ cần báo lỗi có lý do.

Tạo bộ demo gồm A1, A2, A3, A4, quầy check-in, ít nhất hai nhà vệ sinh và một số mục thông tin offline. Pose thực tế lấy từ bản đồ Gazebo mà người dùng đang chạy và được xác minh khi có thể. Khi thiếu bản đồ hoặc pose, chuẩn bị dữ liệu minh họa được gắn nhãn và đánh dấu chưa dùng cho điều hướng thực. Tiếp tục hoàn thiện và kiểm thử bằng fixture ở phạm vi cô lập.

Quy tắc chọn điểm trong một nhóm được thực hiện ở node điều hướng.

1. Lọc điểm enabled, đúng map và có pose hợp lệ.
2. Gọi `ComputePathToPose` của Nav2 từ vị trí hiện tại cho các ứng viên, có timeout và giới hạn số yêu cầu đang chờ.
3. Tính chiều dài đường từ các đoạn của path trả về. Các ứng viên cần xuất phát từ vị trí đủ nhất quán để so sánh.
4. Loại các kết quả planner không tìm được đường. Khi có lỗi toàn hệ thống như mất TF, xử lý lỗi và tránh kết luận mọi địa điểm đều không thể tới.
5. Chọn điểm có chiều dài đường ngắn nhất trong các kết quả hợp lệ. Có quy tắc ổn định khi bằng nhau.
6. Khi timeout khiến kết quả chưa đầy đủ, thể hiện đúng mức chắc chắn, ví dụ chọn điểm có đường ngắn nhất trong các kết quả đã nhận, thay vì tuyên bố đã so sánh đầy đủ mọi điểm.
7. Thông báo tên điểm đã chọn và gửi `NavigateToPose`.

Đích cụ thể cũng được kiểm tra trước khi thực thi. Goal nằm ở vị trí robot có thể đứng để trả khách trong vùng đi được. Path có trước khi xuất phát vẫn cần được Nav2 xử lý lại khi môi trường thay đổi.

## 8. Vòng đời nhiệm vụ và phản hồi

Node navigation là nguồn trạng thái chuyến. Thiết kế state machine rõ ràng, có thể dùng các trạng thái IDLE, RESOLVING, ANNOUNCING, STARTING, NAVIGATING, PAUSING, PAUSED, CANCELING và các kết quả kết thúc.

### 8.1. Bắt đầu

Xác minh yêu cầu, dữ liệu, localization và action server. Chọn đích, phát thông báo “Tôi sẽ dẫn bạn đến cổng A3”, đợi đúng câu thông báo hoàn tất nếu bật playback, rồi gửi goal. Dùng id câu phát và ack riêng, tránh suy ra từ một Bool speaking chung. Có timeout và chính sách khi phát tiếng lỗi. Trong chế độ chỉ log hoặc tắt playback, ghi nhận thông báo đã xuất và tiếp tục được.

Thông báo ý định dẫn đi có thể được phát trước goal acceptance. Chỉ báo đang thực hiện chuyến khi action được chấp nhận. Theo dõi feedback và kết quả thật. Kết quả thành công mới phát câu đã tới nơi.

### 8.2. Tạm dừng và tiếp tục

Với API Nav2 đang cài, kiểm tra cách pause phù hợp. Phương án khả thi là hủy goal navigation hiện tại, chờ kết quả hủy, giữ pose đích của mission và chuyển PAUSED. Resume gửi goal mới từ vị trí hiện tại tới đích đã giữ, tiếp tục cùng mission.

Tiếp nhận pause cả khi đang chọn đường, đang phát thông báo hoặc goal đang chờ chấp nhận. Nếu goal được chấp nhận muộn, xử lý hủy đúng goal. Chỉ báo đã tạm dừng sau khi thao tác cần thiết hoàn tất. Mô tả rõ rằng đây là điều khiển nhiệm vụ giọng nói, kết quả dừng cơ khí cần kiểm chứng qua stack điều hướng và robot.

### 8.3. Hủy và đổi đích

Cancel dọn yêu cầu đang chờ, hủy đúng goal, xử lý response và kết quả trước khi kết thúc mission. Khi cancel bị từ chối hoặc timeout, giữ trạng thái lỗi phản ánh thực tế và báo rõ.

Đổi đích phải có chính sách nhất quán. Chọn mặc định xác minh đích mới trước, sau đó hủy chuyến cũ và chờ hoàn tất trước khi bắt đầu đích mới. Nếu đích mới không hợp lệ, chuyến hiện tại tiếp tục và robot thông báo lỗi của yêu cầu mới. Trong PAUSED, đổi đích hợp lệ cập nhật đích đang giữ và tiếp tục ở PAUSED cho đến lệnh resume.

Với yêu cầu mới đến dồn dập, giữ một yêu cầu thay thế mới nhất và trả trạng thái cho yêu cầu bị thay thế. Cancel ưu tiên hơn yêu cầu bắt đầu đang chờ. Mọi callback phải liên kết đúng request, mission và goal id. Callback cũ hoàn tất muộn phải được nhận diện để không ghi đè nhiệm vụ mới.

### 8.4. Lỗi, khởi động lại và thời gian

Xử lý server chưa sẵn sàng, goal bị từ chối, abort, cancel, timeout, cấu hình sai, mất kết nối và feedback lỗi thời. Sau khi node điều khiển bị restart, trạng thái ban đầu phải phản ánh việc chưa biết chuyến cũ có còn chạy hay không cho đến khi đối chiếu được. Runtime này phải có cách thu hồi hoặc xử lý goal còn chạy của phiên trước nếu có. Document rõ giới hạn phục hồi theo API thực tế.

`use_sim_time` được cấu hình cho node cần tương tác mô phỏng và stamp pose. Timeout nhận âm thanh, process và kiểm thử dùng đồng hồ phù hợp để vẫn hoạt động khi Gazebo pause. Kiểm tra namespace, frame, ROS_DOMAIN_ID và action name thực tế, cho phép cấu hình hoặc remap.

## 9. STT, TTS và âm thanh trong Docker

- Nạp model Vosk từ đường dẫn cấu hình. Thông báo rõ thiếu model, sai model, thiếu thư viện, sai thiết bị hoặc sample rate không hỗ trợ.
- Hỗ trợ keyboard và WAV khi micro chưa sẵn sàng. Input WAV có định dạng được kiểm tra hoặc được chuyển đổi rõ ràng.
- Có danh sách thiết bị và cách chọn micro bằng tham số. Kiểm tra quyền và đường âm thanh thực tế của container hiện tại.
- Giới hạn hàng đợi audio, xử lý overflow, reset recognizer và bỏ audio cũ phù hợp quanh lúc TTS để giảm việc robot nghe lại tiếng của chính mình.
- Xác minh chính sách luôn nghe hoặc wake word. Mặc định demo có thể luôn nghe. Nếu bật wake word, có timeout trở lại trạng thái chờ và test riêng.
- TTS giữ Piper và eSpeak. Có chế độ chỉ log để kiểm thử đầy đủ logic khi loa thiếu. Ghi rõ chế độ đang sử dụng.
- Kiểm tra exit code và timeout của subprocess. Quản lý file tạm, playback queue, speaking state và shutdown khi lỗi.
- Lệnh pause/cancel phải xử lý được khi hàng đợi phát tiếng đang bận. Với cơ chế tạm ngưng nghe trong lúc TTS, document rõ giới hạn nhận lệnh qua micro trong khoảng đó. Lệnh trên topic vẫn được tiếp nhận.
- Phản hồi hủy hoặc lỗi có thể ưu tiên và loại bỏ câu thông báo cũ chưa phát. Tránh đọc lời bắt đầu chuyến đã bị hủy.
- Chuẩn hóa cách đọc mã cổng cho TTS khi cần, ví dụ A3 đọc là “a ba”.
- Tiếp tục làm các phần độc lập khi môi trường không có thiết bị âm thanh. Chỉ ghi đạt cho kiểm thử micro/loa khi có bằng chứng thực tế.

Nếu cần sửa cấu hình Docker để truy cập audio, xác minh cấu hình và thiết bị hiện tại, ghi lại thay đổi nhỏ nhất. Mọi file mới mang vai trò cấu hình phần mềm được đặt đúng vị trí cấu hình project. Giải thích riêng bước nào đòi hỏi người dùng reopen/rebuild container.

## 10. Kiểm thử và tiêu chí nghiệm thu

Bổ sung test hồi quy có giá trị để bắt đúng các lỗi đã phát hiện. Test chạy lại được từ package, dùng fixture để bảo đảm ổn định. Tách test giả lập khỏi bài kiểm tra Nav2/Gazebo và thiết bị thật.

| Nhóm | Tình huống tối thiểu |
| --- | --- |
| NLU | Có dấu, không dấu, Unicode tổ hợp, aliases, A3, B2, cổng số 15, mã không tồn tại, A123 không bị cắt thành A12, alias A1 không khớp A10, thiếu đích, câu ngoài phạm vi. |
| Ý nghĩa hành động | Hỏi vị trí tạo yêu cầu dẫn, hỏi thông tin tạo câu trả lời, phủ định không navigate, “không cần nữa” cancel, “không dừng lại” không pause, nhiều đích rõ ràng trong một câu. |
| Dữ liệu | Pose sai, alias trùng, điểm disabled, category nhiều điểm, dữ liệu cũ hợp lệ. |
| Chọn đích | Điểm gần đường thẳng nhưng đường đi dài hơn, điểm không có đường, planner timeout hoặc mất TF, không có kết quả hợp lệ. |
| ROS độc lập | Nhập chữ qua node, echo transcript/intent/response, voice chạy khi Nav2 thiếu, subscriber mới không kích hoạt lệnh cũ. |
| Vòng đời | Goal accepted/rejected, success/abort, pause/resume, cancel trong lúc chờ goal, cancel trong lúc TTS, cancel bị từ chối, đổi đích, kết quả cũ về muộn, yêu cầu lặp cùng id. |
| Khởi động lại | Navigation gateway restart khi đang có chuyến, voice restart đọc lại trạng thái, xử lý trạng thái chưa biết. |
| Audio | Vosk đọc WAV, micro thật, TTS synthesize, phát loa thật, thiếu model, sai device, timeout và lỗi subprocess. |
| Mô phỏng | Goal trên map thực tế, feedback thật, tới đích, tạm dừng và đi tiếp, hủy, đổi đích, Gazebo pause. |

Nhóm test fixture phải cô lập action/topic để không tác động navigation thật. Báo riêng bằng chứng fixture, Gazebo và robot thật. Trước thử chuyển động, xác minh đây là Gazebo đang hoạt động. Với robot thật, cần người dùng xác nhận điều kiện chạy chuyển động.

Trong `PROGRESS.md`, mỗi mục nghiệm thu ghi trạng thái đạt, lỗi hoặc chưa kiểm chứng, cùng lệnh đã chạy, kết quả và giới hạn. Tránh dùng pass do mock để kết luận micro, loa hoặc Nav2 thật đã chạy.

## 11. Trình tự thực hiện

### Giai đoạn 0 — Khảo sát và ghi quy tắc

- Xác minh workspace, Git status, container, ROS_DISTRO, Python, package và dependency.
- Đặt đặc tả tại `.agent/airport_robot_voice/PROJECT_SPEC.md` và tạo `PROGRESS.md` nếu chưa có.
- Viết rule ngắn dùng chung theo phần 2. Kiểm tra skill và discovery theo phần 3.
- Ghi kiến trúc, schema và danh sách công việc trong `PROGRESS.md` trước khi sửa chức năng.

### Giai đoạn 1 — Package và chế độ voice độc lập

- Hoàn thiện metadata, dependency, package share path và entry point.
- Build đúng container, chạy keyboard và các topic của voice khi Nav2 chưa chạy.
- Thiết lập config demo và ý nghĩa rõ ràng của mode chỉ log.

### Giai đoạn 2 — NLU và dữ liệu

- Hoàn thiện toàn bộ intent, phủ định, slot, alias, kiểm tra điểm tồn tại và phản hồi offline.
- Chốt schema request/event/state và test action/target/response.

### Giai đoạn 3 — Navigation và quản lý nhiệm vụ

- Hoàn thiện chọn đích theo planner và vòng đời đầy đủ.
- Test callback race, pause/resume/cancel/replace và restart bằng môi trường cô lập trước khi kết nối mô phỏng.

### Giai đoạn 4 — STT và TTS offline

- Hoàn thiện Vosk, WAV, thiết bị micro, Piper/eSpeak, playback ack, hàng đợi, timeout và shutdown.
- Kiểm tra model, audio container và các nhánh lỗi. Ghi rõ bài nào cần người dùng nói vào micro.

### Giai đoạn 5 — Tích hợp và bàn giao

- Build lại, chạy các test liên quan và demo trên Gazebo khi môi trường sẵn sàng.
- Hoàn thiện README bằng tiếng Việt, lệnh copy/paste thực tế cho từng terminal trong container, tham số, schema, cấu hình điểm đến và xử lý lỗi thường gặp.
- Cập nhật `PROGRESS.md` với kết quả, file thay đổi, việc còn lại và bước tiếp tục cụ thể nếu có trở ngại.

Thực hiện tuần tự, kiểm chứng từng giai đoạn rồi tiếp tục. Các lựa chọn kỹ thuật thông thường được tự xử lý và ghi lại. Khi thiếu micro, model hoặc Gazebo, tiếp tục các chức năng và kiểm thử độc lập đã được cho phép. Khi cần dữ liệu hoặc thao tác mà chỉ người dùng có thể cung cấp, nêu chính xác phần bị chặn và hoàn thành những phần còn làm được.

## 12. Nội dung bàn giao

1. Package local có code đầy đủ và build được trên Jazzy, với các mode và chức năng đã nêu.
2. Rule và các tài liệu AI tập trung tại `.agent`, có đường dẫn cụ thể và cơ chế nạp đã kiểm tra.
3. Danh mục demo, quy tắc thêm địa điểm và cách lấy pose trên map thực tế.
4. Test chạy lại được và kết quả theo từng môi trường.
5. README chứa hướng dẫn terminal cho keyboard, WAV, micro, loa, voice độc lập và kết hợp Nav2.
6. Báo cáo cuối ngắn gọn về thay đổi, kiểm chứng, quyết định kỹ thuật và phần chưa xác minh.

Bắt đầu bằng khảo sát code local và giai đoạn 0, sau đó triển khai đến khi đạt các tiêu chí hoặc gặp trở ngại cụ thể đã được ghi rõ.
