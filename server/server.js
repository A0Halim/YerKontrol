const express = require('express');
const bodyParser = require('body-parser');
const cors = require('cors');

const app = express();
const PORT = 5000;

// Middleware
app.use(cors());
app.use(bodyParser.json());

// Takım bilgileri (test amaçlı)
const teams = {
    "team1": { password: "pass123", token: "team1_token" },
    "team2": { password: "pass456", token: "team2_token" }
};

// Oturum açan takımlar
let authenticatedTeams = {};

// Sunucu saati formatı
function getServerTime() {
    const now = new Date();
    return {
        saat: now.getHours(),
        dakika: now.getMinutes(),
        saniye: now.getSeconds(),
        milisaniye: now.getMilliseconds()
    };
}

// Hava Savunma Sistemi koordinatları (örnek veri)
const hssKoordinatlari = [
    { id: 1, enlem: 40.1234, boylam: 29.5678, aktif: true },
    { id: 2, enlem: 40.2234, boylam: 29.6678, aktif: false }
];

// QR koordinatı (örnek veri)
const qrKoordinati = {
    enlem: 40.9876,
    boylam: 29.5432
};

// 1. Giriş Endpoint'i
app.post('/api/giris', (req, res) => {
    const { kullanici_adi, sifre } = req.body;
    
    if (!kullanici_adi || !sifre) {
        return res.status(400).json({ hata: "Kullanıcı adı ve şifre gereklidir" });
    }
    
    const team = teams[kullanici_adi];
    
    if (team && team.password === sifre) {
        const token = team.token;
        authenticatedTeams[token] = true;
        return res.status(200).json({ token });
    } else {
        return res.status(401).json({ hata: "Geçersiz kullanıcı adı veya şifre" });
    }
});

// 2. Sunucu Saati Endpoint'i
app.get('/api/sunucusaati', (req, res) => {
    const token = req.headers.authorization;
    
    if (!token || !authenticatedTeams[token]) {
        return res.status(401).json({ hata: "Oturum açmanız gerekmektedir" });
    }
    
    res.status(200).json(getServerTime());
});

// 3. Telemetri Gönder Endpoint'i
app.post('/api/telemetri_gonder', (req, res) => {
    const token = req.headers.authorization;
    
    if (!token || !authenticatedTeams[token]) {
        return res.status(401).json({ hata: "Oturum açmanız gerekmektedir" });
    }
    
    const telemetri = req.body;
    
    if (!telemetri || !telemetri.enlem || !telemetri.boylam) {
        return res.status(204).json({ hata: "Geçersiz telemetri formatı" });
    }
    
    // Burada telemetri verilerini işleyebilirsiniz
    // Örnek yanıt - diğer takımların bilgileri
    const response = {
        durum: "OK",
        diger_takimlar: [
            { takim_id: "team2", enlem: 40.1111, boylam: 29.2222 }
        ]
    };
    
    res.status(200).json(response);
});

// 4. Kilitlenme Bilgisi Endpoint'i
app.post('/api/kilitlenme_bilgisi', (req, res) => {
    const token = req.headers.authorization;
    
    if (!token || !authenticatedTeams[token]) {
        return res.status(401).json({ hata: "Oturum açmanız gerekmektedir" });
    }
    
    const kilitlenme = req.body;
    
    if (!kilitlenme || !kilitlenme.hedef_takim || !kilitlenme.kilitlenme_zamani) {
        return res.status(204).json({ hata: "Geçersiz kilitlenme formatı" });
    }
    
    res.status(200).json({ durum: "Kilitlenme kaydedildi" });
});

// 5. Kamikaze Bilgisi Endpoint'i
app.post('/api/kamikaze_bilgisi', (req, res) => {
    const token = req.headers.authorization;
    
    if (!token || !authenticatedTeams[token]) {
        return res.status(401).json({ hata: "Oturum açmanız gerekmektedir" });
    }
    
    const kamikaze = req.body;
    
    if (!kamikaze || !kamikaze.okunan_metin) {
        return res.status(204).json({ hata: "Geçersiz kamikaze formatı" });
    }
    
    res.status(200).json({ durum: "Kamikaze bilgisi kaydedildi" });
});

// 6. QR Koordinatı Endpoint'i
app.get('/api/qr_koordinati', (req, res) => {
    const token = req.headers.authorization;
    
    if (!token || !authenticatedTeams[token]) {
        return res.status(401).json({ hata: "Oturum açmanız gerekmektedir" });
    }
    
    res.status(200).json(qrKoordinati);
});

// 7. HSS Koordinatları Endpoint'i
app.get('/api/hss_koordinatlari', (req, res) => {
    const token = req.headers.authorization;
    
    if (!token || !authenticatedTeams[token]) {
        return res.status(401).json({ hata: "Oturum açmanız gerekmektedir" });
    }
    
    res.status(200).json(hssKoordinatlari);
});

// 404 Handler
app.use((req, res) => {
    res.status(404).json({ hata: "Geçersiz URL" });
});

// Sunucuyu başlat
app.listen(PORT, '127.0.0.25', () => {
    console.log(`Test sunucusu http://127.0.0.25:${PORT} adresinde çalışıyor`);
});