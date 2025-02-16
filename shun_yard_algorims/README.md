# Shunting yard algorithm

Shunting yard algorithm - is a method for parsing arithmetical expressions, specified in infix notation.

Conversion of infix expression to postfix expression is implemented - `Infix_to_Postfix(string_view)`.

Two C++ 17 implementations of the algorithm are shown here.<br />


$${\color{green} 1 var  }$$<br />
`string input1 = "(24.5+ 4)*(3.55-1.23)+ 2.89/ 1.44";`  $${\color{green} = 68.127 }$$<br />
`ShuntingYard sh1;`<br />
`auto res1 = sh1.Calculation(input1);`<br />

$${\color{green}2 var - without &ensp; brackets &ensp; and &ensp; validation &ensp; checks !!!!!! }$$<br />
`string input2 = "25.2 + 36.5 * 21 - 43 ";` $${\color{green} = 748.7}$$<br />
`ShuntingYard_2 sh2;`<br />
`auto res2 = sh2.Calculation(input2);`

