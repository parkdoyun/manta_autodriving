/* eslint-disable prettier/prettier */
/* eslint-disable curly */
export function nameValidator(name) {
  if (!name) return "Name can't be empty.";
  if (name < 2 || name > 10) return '2글자 이상 10글자 미만으로 입력해주세요.';
  return '';
}

export function passwordValidator(password) {
  if (!password) return "Password can't be empty.";
  if (password.length < 8 || password.length > 16)
    return 'Password must be at least 8 characters.';
  return '';
}

export function checkPasswordValidator(password, checkPassword) {
  if (checkPassword !== password) return '비밀번호가 일치하지 않습니다.';
  return '';
}
