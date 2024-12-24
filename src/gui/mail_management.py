import smtplib
from email.mime.text import MIMEText

def send_email(recvEmail, subject, text):
    """
    고정된 발신자 이메일과 비밀번호를 사용하여 이메일을 전송하는 함수.
    
    :param recvEmail: 수신자 이메일
    :param subject: 메일 제목
    :param text: 메일 내용
    """
    sendEmail = "email"  # 발신자 이메일 (고정)
    password = "password"  # 발신자 이메일 비밀번호 (고정)
    
    smtpName = "smtp.naver.com"  # SMTP 서버 주소
    smtpPort = 587  # SMTP 포트 번호
    
    # 이메일 메시지 구성
    msg = MIMEText(text)  # 이메일 본문
    msg['Subject'] = subject  # 이메일 제목
    msg['From'] = sendEmail  # 발신자
    msg['To'] = recvEmail  # 수신자
    
    try:
        # SMTP 서버에 연결하고 이메일 전송
        with smtplib.SMTP(smtpName, smtpPort) as s:
            s.starttls()  # TLS 보안 처리
            s.login(sendEmail, password)  # 로그인
            s.sendmail(sendEmail, recvEmail, msg.as_string())  # 메일 전송
            print("Email sent successfully!")
    except Exception as e:
        print(f"Error: {e}")

