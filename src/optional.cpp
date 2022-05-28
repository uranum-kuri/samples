#include <iostream>
#include <optional>
#include <string>

std::optional<int> division(const int x, const int y) {
    if (y == 0) {
        return std::nullopt;  // error
    }
    return x / y;  // ok
}

void print(const std::optional<std::string>& str) {
    std::cout << str.value_or("null") << std::endl;  // value or default "null"
}

int main() {
    std::optional<int> result1 = division(4, 2);  // ok
    std::optional<int> result2 = division(2, 0);  // error

    if (result1.has_value()) {  // member
        std::cout << "result1 : " << result1.value() << std::endl;
    } else {
        std::cout << "result1 is invalid." << std::endl;
    }

    if (result2) {  // operator bool
        std::cout << "result2 : " << result2.value() << std::endl;
    } else {
        std::cout << "result2 is invalid." << std::endl;
    }

    print("Hello!");      // ok
    print(std::nullopt);  // null

    std::optional<std::string> str =
        std::make_optional<std::string>("Hello!");  // helper

    std::cout << "str : " << *str << std::endl;  // pointer access
    *str = "World";                              // change
    std::cout << "str : " << *str << std::endl;

    if (str.has_value()) {
        std::cout << "str size : " << str->size()  // call member function
                  << std::endl;
    }

    std::optional<int> bad_value;
    try {
        bad_value.value();  // bad access
    } catch (std::bad_optional_access& e) {
        std::cout << "bad value error : " << e.what() << std::endl;
    }

    return 0;
}
