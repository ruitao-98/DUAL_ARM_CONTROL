#include <iostream>

int main() {
    char input;
    bool running = true;
    
    while (running) {
        std::cout << "Enter 1 for case1, 2 for case2, or any other key to exit:" << std::endl;
        std::cin >> input;
        
        switch(input) {
            case '1':
                // 执行第一个case的代码
                std::cout << "Entered Case 1" << std::endl;
                // 这里添加Case 1的逻辑
                break;
            case '2':
                // 执行第二个case的代码
                std::cout << "Entered Case 2" << std::endl;
                // 这里添加Case 2的逻辑
                break;
            default:
                // 如果输入不是1或者2，打印信息并退出循环
                std::cout << "Default case: Random Information" << std::endl;
                running = false; // 设置循环为不运行状态，以退出
                break;
        }
    }
    
    return 0;
}